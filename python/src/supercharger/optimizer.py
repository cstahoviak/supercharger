"""
The Python supercharger optimization module.
"""
from functools import partial
from typing import Sequence
from warnings import warn

import numpy as np
from scipy import optimize

from supercharger.pysupercharger import (
    distance,
    Optimizer,
    PlannerResult
)


def cost_fcn(x: Sequence[float]) -> float:
    """

    Args:
        x: (n-2,) A list of charging durations for all nodes but the first and
            last node.

    Returns: The total sum of charging durations.

    """
    return sum(x)

def get_arrival_range(
        durations: Sequence[float],
        rates: Sequence[float],
        distances: Sequence[float],
        init_arrival_rng: float) -> np.ndarray:
    """
    Computes the arrival range at each node [2, n].

    Args:
        durations: (n-2,) A list of charging durations for all nodes but the
            first and last node, [2, n-1]
        rates: (n-2,) The known charging rates at each node for nodes [2, n-1]
        distances: (n-2,) The known distances between nodes, e.g. distances[0]
            is the distance from the second node to the third node.
        init_arrival_rng: The known arrival range at node two (the node
            following the origin).

    Returns: (n-2,) The arrival range at each node from the third node onward.

    """
    arrival_ranges = np.zeros_like(durations)
    for idx, (duration, rate, dist) in (
            enumerate(zip(durations, rates, distances))):
        # Set the previous arrival range
        prev_arrival_rng = arrival_ranges[idx - 1] if idx else init_arrival_rng

        # Compute the arrival range at the current node.
        arrival_ranges[idx] = prev_arrival_rng + duration * rate - dist

    return arrival_ranges


def ineq_constraint(x: Sequence[float], rates: Sequence[float],
                    distances: Sequence[float], init_arrival_rng: float) -> \
        np.ndarray:
    """
    The inequality constraint applies to all nodes between the second node and
    the second to last node, [2, n-1].
    """
    return get_arrival_range(x, rates, distances, init_arrival_rng)[:-1]


def eq_constraint(x: Sequence[float], rates: Sequence[float],
                  distances: Sequence[float],
                  init_arrival_rng: float) -> np.ndarray:
    """
    The equality constraint applies to the last node - a time optimal route will
    have an arrival range of zero at the final node.
    """
    return get_arrival_range(x, rates, distances, init_arrival_rng)[-1]


class NonlinearOptimizer(Optimizer):
    """
    Implements nonlinear optimization of a given route.
    """
    # def __init__(self):
    #     # If we want to define our own constructor, we must first call the
    #     # bound C++ constructor.
    #     Optimizer.__init__(self)

    def optimize(self, result: PlannerResult) -> PlannerResult:
        """
        The public interface of the Python optimization class.

        Args:
            result: The planning algorithm result to be optimized.

        Returns:
            The optimized route.
        """
        return self.Optimize(result)

    def Optimize(self, result: PlannerResult) -> PlannerResult:
        """
        Optimize the route via SLSQP constrained, nonlinear optimization.
        Args:
            result: The planning algorithm result to be optimized.

        Returns:
            The optimized route.
        """
        # Define some arrays for route optimization
        distances = []
        rates = []
        for idx, node in enumerate(result.route[1:]):
            distances.append(distance(node, result.route[idx]))
            rates.append(node.charger.rate)

        # Define the dimensionality of the problem (n-2)
        dim = len(result.route) - 2

        # Define the lower and upper bounds of the optimization constraint - the
        # arrival range at each node, for nodes [2, n-1].
        ineq_constraint_lb = np.zeros(dim - 1)
        # The upper bound on the arrival range at a given node is equal to the
        # vehicle's maximum range minus the distance between that node and the
        # previous node.
        ineq_constraint_ub = \
            np.full_like(ineq_constraint_lb, result.max_range) - distances[1:-1]

        # Define the nonlinear inequality and equality constraints
        constraints = []
        constraints.append(optimize.NonlinearConstraint(
            fun=partial(ineq_constraint,
                        rates=rates[:-1],
                        distances=distances[1:],
                        init_arrival_rng=result.route[1].arrival_range),
            lb=ineq_constraint_lb,
            ub=ineq_constraint_ub)
        )
        constraints.append(optimize.NonlinearConstraint(
            fun=partial(eq_constraint,
                        rates=rates[:-1],
                        distances=distances[1:],
                        init_arrival_rng=result.route[1].arrival_range),
            lb=0,
            ub=0)
        )

        # Define the bounds on the charging durations.The upper bound for the
        # charge time at each node is determined by the known arrival range at
        # that node.
        ub = np.array([(result.max_range - n.arrival_range) / n.charger.rate
                       for n in result.route[1:-1]])
        bounds = optimize.Bounds(lb=np.zeros_like(ub), ub=ub)

        # Define and run the optimization problem
        optimization_result = optimize.minimize(
            fun=cost_fcn,
            x0=np.array([node.duration for node in result.route[1:-1]]),
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
