"""
A sandbox script for exploring optimization in python.
"""
from functools import partial
from time import perf_counter
from typing import List, Sequence

import numpy as np
from scipy import optimize

from supercharger.pysupercharger import (
    AlgorithmType,
    CostFunctionType,
    distance,
    Node,
    PlannerResult,
    RoutePlanner
)


def cost_fcn(x: List[float]) -> float:
    """

    Args:
        x: (n-2,) A list of charging durations for all nodes but the first and
            last node.

    Returns: The total sum of charging durations.

    """
    return sum(x)


def constraint(x: List[float], rates: List[float], distances: List[float],
               init_arrival_rng: float) -> np.ndarray:
    """

    Args:
        x: (n-2,) A list of charging durations for all nodes but the first and
            last node, [2, n-1]
        rates: (n-2,) The known charging rates at each node for nodes [2, n-1]
        distances: (n-2,) The known distances between nodes, e.g. distances[0]
            is the distance from the second node to the third node.
        init_arrival_rng: The known arrival range at node two (the node
            following the origin).

    Returns: (n-2,) The arrival range at each node from the third node onward.

    """
    # Compute the arrival ranges for node 3 onward
    arrival_ranges = np.zeros(len(x))
    for idx, (duration, rate, dist) in enumerate(zip(x, rates, distances)):
        # Set the previous arrival range
        prev_arrival_rng = arrival_ranges[idx - 1] if idx else init_arrival_rng

        # Compute the arrival range at the current node.
        arrival_ranges[idx] = prev_arrival_rng + duration * rate - dist

    return arrival_ranges

def update_planner_result(result: PlannerResult,
                          durations: Sequence[float]) -> PlannerResult:
    """

    Args:
        result: The initial route planner result.
        durations: The updated set of charging durations at each node, not
            including the first and last nodes.

    Returns:
        The updated planner result.
    """
    if len(durations) != len(result.route) - 2:
        raise ValueError(
            f"The length of the 'new_durations' array must be equal to two "
            f"less than len(PlannerResult.route) ({len(result.route) - 2}). "
            f"Actual length is {len(durations)}.")

    # Update the charging duration for all nodes between the start and end.
    updated = result
    for node, duration in zip(updated.route[1:-1], durations):
        node.duration = duration

    # Update the arrival and departure ranges and the cost at each node
    for idx, node in enumerate(updated.route[1:]):
        previous = updated.route[idx]

        # Update the arrival range at the current node
        node.arrival_range = previous.arrival_range + \
            previous.duration * previous.charger.rate - distance(previous, node)

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


if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the route planner using Dijkstra's algorithm
    planner = RoutePlanner(algo_type=AlgorithmType.DIJKSTRAS)

    # Set the vehicle's speed and max range
    planner.max_range = 320
    planner.speed = 105

    # Plan the route with Dijkstra's algorithm
    start = perf_counter()
    result = planner.plan_route(origin, destination)
    stop = perf_counter()
    print(f"Route Planning time: {(stop - start) * 1e3:.2f} ms")
    print(f"Dijkstra's Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Print some debug info
    for idx, node in enumerate(result.route):
        print(f"{node.name} arrival range: {node.arrival_range:.4f}")
        print(f"{node.name} rate: {node.charger.rate}")
        if idx < len(result.route) - 1:
            print(f"{node.name} -> {result.route[idx + 1].name} distance: "
                  f"{distance(node, result.route[idx + 1]):.4f}")

    # Define some arrays for route optimization
    distances = []
    durations = []
    rates = []
    for idx, node in enumerate(result.route[1:]):
        distances.append(distance(node, result.route[idx]))
        durations.append(node.duration)
        rates.append(node.charger.rate)

    # Define the upper and lower bounds of the optimization constraint - the
    # arrival range at each node, for all nodes [2, n]
    n_nodes = len(result.route)
    constraint_lb = np.zeros(n_nodes - 2)
    # The upper bound on the arrival range at a given node is equal to the
    # vehicle's maximum range minus the distance between that node and the
    # previous node.
    constraint_ub = np.full(n_nodes - 2, result.max_range) - distances[1:]
    # The arrival range at the last node should be zero.
    constraint_ub[-1] = 0

    func = partial(constraint,
                   rates=rates[:-1],
                   distances=distances[1:],
                   init_arrival_rng=result.route[1].arrival_range)
    arrival_ranges = func(durations[:-1])
    np.testing.assert_array_equal(arrival_ranges, np.zeros_like(arrival_ranges))

    # Define the bounds on the charging durations
    # TODO: Could actually compute an upper boud for the charge duration for
    #  each node.
    bounds = optimize.Bounds(lb=0, ub=np.inf)

    # Define the nonlinear constraint
    nl_constraint = optimize.NonlinearConstraint(
        fun=partial(constraint,
                   rates=rates[:-1],
                   distances=distances[1:],
                   init_arrival_rng=result.route[1].arrival_range),
        lb=constraint_lb,
        ub=constraint_ub
    )

    # Define the optimization problem
    # TODO: Address "equality and inequality constraints are specified in the
    #  same element of the constraint list." warning. See the following link:
    #  https://github.com/scipy/scipy/issues/12310
    start = perf_counter()
    optimization_result = optimize.minimize(
        fun=cost_fcn,
        x0=np.array([node.duration for node in result.route[1:-1]]),
        bounds=bounds,
        constraints=nl_constraint)
    stop = perf_counter()
    print(f"Optimization time: {(stop - start) * 1e3:.2f} ms")

    # Did it work?
    if optimization_result.success:
        print(f"New charging durations: {optimization_result.x}")
    else:
        print(optimization_result.message)

    # Create the newly-optimized route
    optimized = update_planner_result(result, optimization_result.x)
    for node in optimized.route:
        print(node.__repr__())
    print(f"Optimized Final Route (Cost: {result.cost:.4f} hrs)")
    print(optimized)
