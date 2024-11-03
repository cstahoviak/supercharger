"""
A Statistical evaluation of the following planning algorithms against the
provided "reference" result.

1. Dijkstra's algorithm using a simple/"naive" cost function.
2. Dijkstra's algorithm (from 1) + a post-planning optimization step.
3.  Dijkstra's algorithm using an "optimized" (same optimization scheme as 2)
    cost function.

Notes:
    1. I made an effort to parallelize the execution of these algorithms
    across all n_samples datapoints, but the ThreadPoolExecutor requires that
    the type provided is pickleable, and the bound C++ classes in use (
    Supercharger, Dijkstras, NLOptimizer) are not pickleable. pybind11 provides
    support for making custom types pickleable, but implementing this seems to
    require public accessors for all protected/private members, and I'm not sure
    if that's in line with the design of those classes.
"""
from collections import OrderedDict
from concurrent.futures import ProcessPoolExecutor
from datetime import timedelta
from functools import partial
from time import perf_counter
import os

import numpy as np

from supercharger.optimize import optimized_cost
from supercharger.utils.logger import get_logger
from supercharger.utils.paths import supercharger_build, supercharger_root
from supercharger.utils.subprocess import run_executable
from supercharger.types import AlgoStats

from pysupercharger import (
    dijkstras_simple_cost,
    distance,
    Planner,
    OptimizerType,
    Supercharger
)

# Get the logger
logger = get_logger(os.path.basename(__file__))

def get_reference_result(origin: str, destination: str) -> AlgoStats:
    """
    Args:
        origin: The origin node name.
        destination: The destination node name.

    Returns:
        An AlgoStats instance containing the 'reference' cost of the route.
    """
    # Capture the "reference result" cost
    route = run_executable(str(supercharger_path), origin, destination)
    output = run_executable(str(checker_path), route)
    reference_cost = float(output.split('\n')[1].split()[-1])
    return AlgoStats(time=np.nan, cost=reference_cost)


def plan_route(planner: Planner, endpoints: tuple[str, str]) -> AlgoStats:
    """

    Args:
        planner:
        endpoints:

    Returns:

    """
    start = perf_counter()
    result = planner.plan_route(endpoints[0], endpoints[1])
    stop = perf_counter()
    return AlgoStats(time=stop-start, cost=result.cost)


if __name__ == "__main__":
    # Define some statistical variables
    n_samples = 10

    # Set some vehicle parameters
    max_range = 320
    speed = 105

    planner_descriptions = {
        "planner_1": "Dijkstra's\t\t\t\t\t",
        "planner_2": "Dijkstra's + Post Optimization",
        "planner_3": "Diksrtra's + Optimized Cost\t"
    }

    # Create the planners
    planners = OrderedDict()
    planners["planner_1"] = Supercharger(cost_f=dijkstras_simple_cost)
    planners["planner_2"] = Supercharger(cost_f=dijkstras_simple_cost,
                                         optim_type=OptimizerType.NLOPT)
    planners["planner_3"] = Supercharger(cost_f=optimized_cost)

    # Set the max range and speed for each planner
    for planner in planners.values():
        planner.max_range = max_range
        planner.speed = speed

    # Choose a collection of random destinations from the network.
    network = planners["planner_1"].network
    cities = list(network.keys())
    endpoints = []
    while len(endpoints) < n_samples:
        choice = np.random.choice(cities, size=2)
        if distance(network[choice[0]], network[choice[1]]) > 4 * max_range:
            endpoints.append(tuple(choice))

    # Define the application paths
    supercharger_path = supercharger_build() / 'supercharger'
    checker_path = supercharger_root() / 'checker_linux'

    results = OrderedDict()
    # NOTE: This worked for calling get_reference_result()
    with ProcessPoolExecutor() as executor:
        reference_result = executor.map(get_reference_result, endpoints)
        results["Reference"] = [result for result in reference_result]

    # NOTE: This did not work because the Supercharger class is not pickleable.
    for name, planner in planners.items():
        with ProcessPoolExecutor() as executor:
            func = partial(plan_route, planner=planner)
            result = executor.map(func, endpoints)
            results[name] = [item for item in result]

    times0 = np.zeros((n_samples, len(planners) + 1))
    costs0 = np.zeros((n_samples, len(planners) + 1))
    cost_diff0 = np.zeros((n_samples, len(planners)))
    for m, (name, result) in enumerate(results.items()):
        logger.info(f"Unpacking data from '{name}' planner.")
        # 'result' is a generator and generators cannot be iterated over more
        # than once
        result = list(result)
        times0[:, m] = np.array([stat.time for stat in result])
        costs0[:, m] = np.array([stat.cost for stat in result])


    stats = []
    distances = []
    for m, (origin, destination) in enumerate(endpoints):
        logger.info(f"({m}) Planning route between '{origin}' and "
                    f"'{destination}'.")

        distances.append(distance(network[origin], network[destination]))
        route_stats = [get_reference_result((origin, destination))]

        for n, (name, planner) in enumerate(planners.items()):
            # print(f"\t{n}. Planning route with '{planner_descriptions[name]}'.")
            start = perf_counter()
            result = planner.plan_route(origin, destination)
            stop = perf_counter()
            route_stats.append(AlgoStats(time=stop -start, cost=result.cost))

        stats.append(route_stats)

    # Extract the timing and cost data
    times = np.zeros((n_samples, len(planners)))
    costs = np.zeros((n_samples, len(planners) + 1))
    cost_diff = np.zeros((n_samples, len(planners)))
    for m, route_stats in enumerate(stats):
        times[m] = np.array([stat.time for stat in route_stats[1:]])
        costs[m] = np.array([stat.cost for stat in route_stats])

        cost_diff[m, 0] = (route_stats[0].cost - route_stats[1].cost) * 60
        cost_diff[m, 1] = (route_stats[0].cost - route_stats[2].cost) * 60
        cost_diff[m, 2] = (route_stats[0].cost - route_stats[3].cost) * 60

    # Convert times to milliseconds
    times = times * 1e3

    # Output cost statistics
    print("\n\tPLANNER COSTS (relative to the reference result) [mins]")
    print(f'Planning Algorithm\t\t\t\tmean \t std\t max\t min')
    for idx, name in enumerate(planners.keys()):
        print(f'{planner_descriptions[name]}\t'
              f'{cost_diff.mean(axis=0)[idx]:.2f}\t '
              f'{cost_diff.std(axis=0)[idx]:.2f}\t '
              f'{cost_diff.max(axis=0)[idx]:.2f}\t '
              f'{cost_diff.min(axis=0)[idx]:.2f}')

    argmax = cost_diff[:, -1].argmax()
    time_saved = timedelta(hours=(costs[argmax, 0] - costs[argmax, -1]))
    print(f"\nThe greatest time savings among the sampled routes was for the "
          f"route between '{endpoints[argmax][0]}' and "
          f"'{endpoints[argmax][1]}'.\n"
          f"Reference Result: {costs[argmax, 0]:.4f}\n"
          f"Planner Result: {costs[argmax, -1]:.4f}\n"
          f"Time Saved: {str(time_saved).split('.')[0]}")

    # Output profiling statistics
    print("\n\t\t\t\t\t\tPLANNER PROFILING [ms]")
    print(f'Planning Algorithm\t\t\t\tmean \t std\t max\t\t min')
    for idx, name in enumerate(planners.keys()):
        print(f'{planner_descriptions[name]}\t'
              f'{times.mean(axis=0)[idx]:.2f}\t '
              f'{times.std(axis=0)[idx]:.2f}\t '
              f'{times.max(axis=0)[idx]:.2f}\t\t '
              f'{times.min(axis=0)[idx]:.2f}')