"""
The Python supercharger optimization module.
"""
from functools import partial
from typing import Sequence
from warnings import warn

import numpy as np
from scipy import optimize

from python.sandbox.optimization import constraint
from supercharger.optimize.constraints import (
    ConstraintData,
    ineq_constraint,
    ineq_constraint_grad,
    eq_constraint,
    eq_constraint_grad
)

from supercharger.pysupercharger import (
    distance,
    Optimizer,
    PlannerResult
)


def objective(x: Sequence[float]) -> float:
    """
    Args:
        x: (n-2,) A list of charging durations for all nodes but the first and
            last node.

    Returns: The total sum of charging durations.

    """
    return np.sum(x)


def objective_grad(x: Sequence[float]) -> np.ndarray:
    """
    Evaluates the gradient of the cost function as the point x.

    Args:
        x: The evaluation point.

    Returns:
        The cost function gradient evaluated at the point x.
    """
    return np.ones_like(x)


class NonlinearOptimizer(Optimizer):
    """
    Implements nonlinear optimization of a given route.
    """
    def __init__(self):
        # If we want to define our own constructor, we must first call the
        # bound C++ constructor.
        Optimizer.__init__(self)

        # Store the optimization method
        self._available_methods = ['COBYQA', 'SLSQP', 'trust-constr']
        self._method = None

        self._use_linear_constraints = True

    def optimize(self, result: PlannerResult, method: str = 'SLSQP') -> \
            PlannerResult:
        """
        The public interface of the Python optimization class.

        Args:
            result: The planning algorithm result to be optimized.
            method: The type of optimization solver used by scipy.minimize.
            Available methods for constrained, nonlinear optimization are
            'COBYQA', 'SLSQP' and 'trust-constr'.

        Returns:
            The optimized route.
        """
        if method in self._available_methods:
            self._method = method
        else:
            warn(f"Cannot perform constrained, nonlinear optimization via the "
                 f"'{method}' method. Available methods are "
                 f"{self._available_methods}. Proceeding with 'SLSQP'.")
            self._method = 'SLSQP'

        return self.Optimize(result)

    @staticmethod
    def create_constraint_data(result: PlannerResult) -> ConstraintData:
        """
        Create the ConstraintData from a PlannerResult.
        """
        # Extract some relevant data from the planner result.
        distances = []
        rates = []
        for idx, node in enumerate(result.route[1:]):
            distances.append(distance(node, result.route[idx]))
            rates.append(node.charger.rate)

        # Create the constraint data and remove values that do not affect the
        # optimization, i.e. the distance between the first and second nodes,
        # and final node's charging rate.
        return ConstraintData(
            distances=np.array(distances[1:]),
            rates=np.array(rates[:-1]),
            init_arrival_range=result.route[1].arrival_range)

    def Optimize(self, result: PlannerResult) -> PlannerResult:
        """
        Optimize the route via constrained, nonlinear optimization.

        Args:
            result: The planning algorithm result to be optimized.

        Returns:
            The optimized route.
        """
        # Create the constraint data
        constr_data = self.create_constraint_data(result)

        # Define the dimensionality of the problem (n-2)
        dim = len(result.route) - 2

        constraints = []
        if self._use_linear_constraints:
            # Define the linear inequality constraint bounds.
            L = np.tri(dim - 1)
            I = np.eye(dim - 1)
            d = constr_data.distances[:-1]
            ineq_constraint_lb2 = L.dot(d) - constr_data.init_arrival_range
            ineq_constraint_ub2 = np.subtract(L, I).dot(d) + \
                result.max_range - constr_data.init_arrival_range

            # Formulate the inequality constraint as a linear constraint
            constraints.append(optimize.LinearConstraint(
                A=constr_data.A_ineq,
                lb=ineq_constraint_lb2,
                ub=ineq_constraint_ub2)
            )

            # Formulate the equality constraint as a linear constraint
            eq_constraint_lb = constr_data.distances.sum() - \
                               constr_data.init_arrival_range
            eq_constraint_ub = eq_constraint_lb
            constraints.append(optimize.LinearConstraint(
                A=constr_data.A_eq,
                lb=eq_constraint_lb,
                ub=eq_constraint_ub)
            )
        else:
            # Use "nonlinear" constraints.
            # Define the lower and upper bounds of the optimization constraint -
            # the arrival range at each node, for nodes [2, n-1].
            ineq_constraint_lb = np.zeros(dim - 1)
            # The upper bound on the arrival range at a given node is equal to
            # the vehicle's maximum range minus the distance between that node
            # and the previous node.
            ineq_constraint_ub = \
                np.full_like(ineq_constraint_lb, result.max_range) - \
                constr_data.distances[:-1]

            # Define the nonlinear inequality and equality constraints
            constraints.append(optimize.NonlinearConstraint(
                fun=partial(ineq_constraint, constr_data=constr_data),
                lb=ineq_constraint_lb,
                ub=ineq_constraint_ub,
                jac=partial(ineq_constraint_grad, constr_data=constr_data))
            )
            constraints.append(optimize.NonlinearConstraint(
                fun=partial(eq_constraint, constr_data=constr_data),
                lb=0,
                ub=0,
                jac=partial(eq_constraint_grad, constr_data=constr_data))
            )

        # Define the bounds on the charging durations.The upper bound for the
        # charge time at each node is determined by the known arrival range at
        # that node.
        ub = np.array([(result.max_range - n.arrival_range) / n.charger.rate
                       for n in result.route[1:-1]])
        bounds = optimize.Bounds(lb=np.zeros_like(ub), ub=ub)

        # Define and run the optimization problem
        optimization_result = optimize.minimize(
            fun=objective,
            x0=np.array([node.duration for node in result.route[1:-1]]),
            method=self._method,
            jac=objective_grad,
            bounds=bounds,
            constraints=constraints)

        # Did it work?
        if optimization_result.success:
            return self._create_optimized_result(result, optimization_result.x)
        else:
            warn(f"The optimization failed with error message: "
                 f"{optimization_result.message}")
            return result

    @staticmethod
    def _create_optimized_result(result: PlannerResult,
                                 new_durations: Sequence[float]) \
            -> PlannerResult:
        """
        Args:
            result: The initial route planner result.
            new_durations: The updated set of charging durations at each
                node, not including the first and last nodes.

        Returns:
            The updated planner result.
        """
        if len(new_durations) != len(result.route) - 2:
            raise ValueError(
                f"The length of the 'new_durations' array must be equal to two "
                f"less than len(PlannerResult.route) ({len(result.route) - 2}). "
                f"Actual length is {len(new_durations)}.")

        # TODO: Creating a copy of the route doesn't work (updated ends up
        #  becoming an alias of result. Do I need to implement a copy
        #  constructor for the PlannerResult class to fix this or is this
        #  just how python works?
        # updated = result
        updated = PlannerResult()
        updated.route = result.route
        updated.max_range = result.max_range
        updated.speed = result.speed

        # Update the charging duration for all nodes between the start and end.
        for node, duration in zip(updated.route[1:-1], new_durations):
            node.duration = duration

        # Update the arrival and departure ranges and the cost at each node
        for idx, node in enumerate(updated.route[1:]):
            previous = updated.route[idx]

            # Update the arrival range at the current node
            node.arrival_range = previous.arrival_range + \
                                 previous.duration * previous.charger.rate - \
                                 distance(previous, node)

            # Update the cost at the current node
            node.cost = previous.cost + previous.duration + \
                        distance(previous, node) / result.speed

            # For all nodes but the final node, update the departure range
            if idx < len(result.route) - 1:
                node.departure_range = node.arrival_range + \
                                       node.duration * node.charger.rate

        # Finally, update the total cost of the route
        updated.cost = updated.route[-1].cost
        return updated
