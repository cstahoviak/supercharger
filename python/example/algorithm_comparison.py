"""
Recreates the cpp/app/algorithm_comparison.cpp" app on the python side.
"""
from collections import OrderedDict, namedtuple
from time import perf_counter

from supercharger.optimize import optimized_cost

from pysupercharger import (
    dijkstras_simple_cost,
    NaiveCostType,
    OptimizerType,
    Supercharger
)

AlgoStats = namedtuple('AlgoStats', ['time', 'cost'])


if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # These endpoints will show a cost difference between the final two algos.
    # origin = "Burlington_WA"
    # destination = "Marathon_FL"

    # Store a mapping of algorithms and route costs
    algorithms = OrderedDict()

    # Create the supercharger using Dijkstra's algorithm
    supercharger = Supercharger(
        naive_cost_type=NaiveCostType.MINIMIZE_TIME_REMAINING)

    # Set the vehicle's speed and max range
    supercharger.max_range = 320
    supercharger.speed = 105

    # Plan the route via the "Naive" route planner.
    start = perf_counter()
    result1 = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    algorithms["Naive\t\t\t\t\t\t"] = \
        AlgoStats(time=stop - start, cost=result1.cost)

    # Plan the route via Dijkstra's algorithm.
    supercharger.set_planner(cost_f=dijkstras_simple_cost)
    start = perf_counter()
    result2 = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    algorithms["Dijkstra's\t\t\t\t\t"] = \
        AlgoStats(time=stop - start, cost=result2.cost)

    # Improve on Dijkstra's via the "NLOPT" optimizer.
    supercharger.set_optimizer(type=OptimizerType.NLOPT)
    start = perf_counter()
    result3 = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    algorithms["Dijkstra's + Post Optimization"] = \
        AlgoStats(time=stop - start, cost=result3.cost)

    # Use Dijkstra's algorithm with an optimized cost function
    supercharger.set_planner(cost_f=optimized_cost)
    supercharger.set_optimizer(type=OptimizerType.NONE)
    start = perf_counter()
    result4 = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    algorithms["Dijkstra's + Optimized Cost\t"] = \
        AlgoStats(time=stop - start, cost=result4.cost)

    print("\nPlanning Algorithm\t\t\t\tTime [ms]\tCost [hrs]")
    for algo_name, stats in algorithms.items():
        print(f'{algo_name}\t{stats.time * 1e3:.2f}\t\t{stats.cost:.4f}')
