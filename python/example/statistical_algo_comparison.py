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
from time import perf_counter
import os

import numpy as np

from supercharger.optimize import optimized_cost
from supercharger.utils.logger import get_logger
from supercharger.utils.paths import supercharger_build, supercharger_root
from supercharger.utils.plotting import plot_cost_vs_distance
from supercharger.utils.subprocess import run_executable
from supercharger.types import AlgoStats

from pysupercharger import (
    AlgorithmType,
    CostFunctionType,
    distance,
    DijkstrasPlanner,
    Planner,
    OptimizerType,
    Supercharger
)

# Get the logger
logger = get_logger(os.path.basename(__file__))

# Set some module-level variables.
_planner = DijkstrasPlanner(cost_type=CostFunctionType.DIJKSTRAS_SIMPLE)
_max_range = 320
_speed = 105


def get_reference_result(endpoints: tuple[str, str]) -> AlgoStats:
    """
    Args:
        endpoints: A tuple containing the start and endpoints of the route.

    Returns:
        An AlgoStats instance containing the 'reference' cost of the route.
    """
    # Plan the route via Dijkstra's algorithm
    result = _planner.plan(endpoints[0], endpoints[1], _max_range, _speed)

    # Capture the "reference result" cost
    output = run_executable(str(checker_path), str(result))
    reference_cost = float(output.split('\n')[1].split()[-1])
    return AlgoStats(time=np.nan, cost=reference_cost)


if __name__ == "__main__":
    # Define some statistical variables
    n_samples = 2000

    planner_descriptions = {
        "reference": "Reference",
        "planner_1": "Dijkstra's\t\t\t\t\t",
        "planner_2": "Dijkstra's + Post Optimization",
        "planner_3": "Dijkstra's + Optimized Cost\t"
    }

    # Create the planners
    planners = OrderedDict()
    planners["planner_1"] = Supercharger(
        cost_type=CostFunctionType.DIJKSTRAS_SIMPLE)
    planners["planner_2"] = Supercharger(
        cost_type=CostFunctionType.DIJKSTRAS_SIMPLE,
        optim_type=OptimizerType.NLOPT)
    planners["planner_3"] = Supercharger(
        # algo_type=AlgorithmType.DIJKSTRAS,
        # cost_f=optimized_cost,
        cost_type=CostFunctionType.DIJKSTRAS_OPTIMIZED
    )

    # Set the max range and speed for each planner
    for planner in planners.values():
        planner.max_range = _max_range
        planner.speed = _speed

    # Choose a collection of random destinations from the network.
    start = perf_counter()
    network = planners["planner_1"].network
    cities = list(network.keys())
    endpoints = set()
    while len(endpoints) < n_samples:
        choice = np.random.choice(cities, size=2)
        if distance(network[choice[0]], network[choice[1]]) > 3 * _max_range:
            endpoints.add(tuple(choice))
    endpoints = list(endpoints)
    end = perf_counter()
    logger.info(f"{n_samples} unique routes selected in "
                f"{(end - start) * 1e3:.2f} ms.")

    # Define the application paths
    supercharger_path = supercharger_build() / 'supercharger'
    checker_path = supercharger_root() / 'checker_linux'

    # Create the results dict.
    results = OrderedDict()
    for name in planners.keys():
        results[name] = []

    # Process the reference results in batch via parallel processing.
    start = perf_counter()
    with ProcessPoolExecutor() as executor:
        reference_result = executor.map(get_reference_result, endpoints)
        results["reference"] = [result for result in reference_result]
    results.move_to_end("reference", last=False)
    end = perf_counter()
    logger.info(f"Reference result for all {n_samples} routes computed in "
          f"{(end - start):.2f} secs.\n")

    for m, (origin, destination) in enumerate(endpoints):
        logger.info(f"({m}) Planning route between '{origin}' and "
                    f"'{destination}'.")

        for n, (name, planner) in enumerate(planners.items()):
            # logger.info(f"Planning route with "
            #             f"'{planner_descriptions[name].strip('t')}'.")
            start = perf_counter()
            result = planner.plan_route(origin, destination)
            stop = perf_counter()
            results[name].append(AlgoStats(time=stop-start, cost=result.cost))

    # Extract the timing and cost data
    times = np.zeros((n_samples, len(results)))
    costs = np.zeros((n_samples, len(results)))
    cost_diff = np.zeros((n_samples, len(results) - 1))
    for m, algo_stats in enumerate(results.values()):
        times[:, m] = np.array([stat.time for stat in algo_stats])
        costs[:, m] = np.array([stat.cost for stat in algo_stats])

    # Compute difference between the reference result and each planner.
    cost_diff[:, 0] = (costs[:, 0] - costs[:, 1]) * 60
    cost_diff[:, 1] = (costs[:, 0] - costs[:, 2]) * 60
    cost_diff[:, 2] = (costs[:, 0] - costs[:, 3]) * 60

    # Remove nans (from reference result) and convert times to milliseconds.
    times = times[:, 1:] * 1e3

    def float_mins_to_str(value: float) -> str:
        if value > 0:
            return str(timedelta(minutes=value)).split('.')[0]
        else:
            return f"-{str(timedelta(minutes=-value)).split('.')[0]}"

    # Output cost statistics
    print("\n\tPLANNER COSTS (relative to the reference result) [mins]")
    print(f'Planning Algorithm\t\t\t\tmean\t std\t\t max\t\t min')
    for idx, name in enumerate(planners.keys()):
        print(f"{planner_descriptions[name]}\t"
              f"{float_mins_to_str(cost_diff.mean(axis=0)[idx])}\t"
              f" {float_mins_to_str(cost_diff.std(axis=0)[idx])}\t"
              f" {float_mins_to_str(cost_diff.max(axis=0)[idx])}\t"
              f" {float_mins_to_str(cost_diff.min(axis=0)[idx])}")

    argmax = cost_diff[:, -1].argmax()
    time_saved = timedelta(hours=(costs[argmax, 0] - costs[argmax, -1]))
    print(f"\nThe greatest time savings among the {n_samples} sampled routes "
          f"was for the route between '{endpoints[argmax][0]}' and "
          f"'{endpoints[argmax][1]}'.\n"
          f"Reference Result: {costs[argmax, 0]:.4f}\n"
          f"Planner Result: {costs[argmax, -1]:.4f}\n"
          f"Time Saved: {str(time_saved).split('.')[0]}")

    # Output profiling statistics
    print("\n\t\t\t\t\t\tPLANNER PROFILING [ms]")
    print(f'Planning Algorithm\t\t\t\tmean\t std\t\t max\t\t min')
    for idx, name in enumerate(planners.keys()):
        print(f'{planner_descriptions[name]}\t'
              f'{times.mean(axis=0)[idx]:.3f}\t'
              f' {times.std(axis=0)[idx]:.3f}\t\t'
              f' {times.max(axis=0)[idx]:.3f}\t\t'
              f' {times.min(axis=0)[idx]:.3f}\t')

    # Get route distance for all start and endpoints.
    distances = np.array(
        [distance(network[p1], network[p2]) for p1, p2 in endpoints])

    # Plot route costs as a function of distance
    # labels = [name.strip('\t') for name in planner_descriptions.values()]
    # plot_cost_vs_distance(distances, costs, labels)

