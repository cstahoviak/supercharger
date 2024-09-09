"""
A sandbox script for exploring optimization in python.
"""
from functools import partial
from time import perf_counter
from typing import List

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
        # print(f"arrival_range = {prev_arrival_rng:.4f} + {duration:.4f} * "
        #       f"{rate:.4f} - {dist:.4f} = {arrival_ranges[idx]:.4f}")


    return arrival_ranges


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
    # TODO: Could be more specific than np.inf
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
    optimized2 = result
    for node, new_duration in zip(optimized2.route[1:-1], optimization_result.x):
        node.duration = new_duration
        # node.__repr__()
    optimized2.cost = sum([node.duration for node in optimized2.route])
    print(optimized2)
