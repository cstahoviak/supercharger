"""
A demonstration of the Python route optimizer.
"""
from time import perf_counter

from supercharger.optimizer import NonlinearOptimizer

from supercharger.pysupercharger import (
    AlgorithmType,
    RoutePlanner
)


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

    # Optimize the route
    optimizer = NonlinearOptimizer()
    start = perf_counter()
    optimized = optimizer.Optimize(result)
    stop = perf_counter()
    print(f"Optimization time: {(stop - start) * 1e3:.2f} ms")
    print(f"Optimized Final Route (Cost: {optimized.cost:.4f} hrs)")
    print(optimized)
