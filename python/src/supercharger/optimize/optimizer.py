"""
The Python supercharger optimization module.
"""
from functools import partial
from typing import List
from warnings import warn

import numpy as np
import scipy

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


def objective(x: np.ndarray) -> float:
    """
    Args:
        x: (n-2,) An array of charging durations for all nodes but the first and
            last node.

    Returns: The total sum of charging durations.

    """
    return x.sum()


def objective_grad(x: np.ndarray) -> np.ndarray:
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
            warn(f"Cannot perform constrained optimization via the '{method}' "
                 f"method. Available methods are {self._available_methods}. "
                 f"Proceeding with 'SLSQP'.")
            self._method = 'SLSQP'

        return self.Optimize(result)

    def Optimize(self, result: PlannerResult) -> PlannerResult:
        """
        Optimize the route via constrained, nonlinear optimization.

        Args:
            result: The planning algorithm result to be optimized.

        Returns:
            The optimized route.
        """
        # Create the constraints
        constraints = self._create_linear_constraints(
            self.create_constraint_data(result))

        # Define the bounds on the charging durations.The upper bound for the
        # charge time at each node is determined by the known arrival range at
        # that node.
        ub = np.array([(result.max_range - n.arrival_range) / n.charger.rate
                       for n in result.route[1:-1]])
        bounds = scipy.optimize.Bounds(lb=np.zeros_like(ub), ub=ub)

        # Define and run the optimization problem
        optimization_result = scipy.optimize.minimize(
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
            init_arrival_range=result.route[1].arrival_range,
            max_range=result.max_range)

    @staticmethod
    def _create_linear_constraints(data: ConstraintData) -> \
        List[scipy.optimize.LinearConstraint]:
        """
        Args:
            data: The constraint data.

        Returns:
            constraints: A list of constraints - one for the inequality
                constraint on nodes [2, n-1], and one for the equality
                constraint on the final node.
        """
        # Define the linear inequality constraint bounds
        L = np.tri(data.n - 1)
        I = np.eye(data.n - 1)
        d = data.distances[:-1]
        ineq_lb= L.dot(d) - data.init_arrival_range
        ineq_ub = np.subtract(L, I).dot(d) + data.max_range - \
            data.init_arrival_range

        # Formulate the inequality constraint as a linear constraint
        constraints = []
        constraints.append(scipy.optimize.LinearConstraint(
            A=data.A_ineq,
            lb=ineq_lb,
            ub=ineq_ub)
        )

        # Formulate the equality constraint as a linear constraint
        constraints.append(scipy.optimize.LinearConstraint(
            A=data.A_eq,
            lb=data.distances.sum() - data.init_arrival_range,
            ub=data.distances.sum() - data.init_arrival_range)
        )

        return constraints

    @staticmethod
    def _create_nonlinear_constraints(data: ConstraintData) -> \
            List[scipy.optimize.NonlinearConstraint]:
        """
        (Unused) The constraint function is more accurately formulated as a
        linear constraint (see NonlinearOptimizer._create_linear_constraints),
        therefore this method is unused and exists solely as a validation of
        the C++ implementation of the constraint functions.

        Args:
            data: The constraint data.

        Returns:
            constraints: A list of constraints - one for the inequality
                constraint on nodes [2, n-1], and one for the equality
                constraint on the final node.
        """
        # Define the lower and upper bounds of the optimization constraint -
        # the arrival range at each node, for nodes [2, n-1].
        ineq_lb = np.zeros(data.n - 1)
        # The upper bound on the arrival range at a given node is equal to
        # the vehicle's maximum range minus the distance between that node
        # and the previous node.
        ineq_ub = np.full_like(ineq_lb, data.max_range) - data.distances[:-1]

        # Define the nonlinear inequality and equality constraints
        constraints = []
        constraints.append(scipy.optimize.NonlinearConstraint(
            fun=partial(ineq_constraint, constr_data=data),
            lb=ineq_lb,
            ub=ineq_ub,
            jac=partial(ineq_constraint_grad, constr_data=data))
        )
        constraints.append(scipy.optimize.NonlinearConstraint(
            fun=partial(eq_constraint, constr_data=data),
            lb=0,
            ub=0,
            jac=partial(eq_constraint_grad, constr_data=data))
        )

        return constraints


    @staticmethod
    def _create_optimized_result(result: PlannerResult,
                                 new_durations: np.ndarray) -> PlannerResult:
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
