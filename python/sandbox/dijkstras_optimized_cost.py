"""
A demonstration of using an "optimized" cost function with Dijkstra's algorithm.
"""
from time import perf_counter

from supercharger.optimize import optimized_cost

from pysupercharger import (
    dijkstras_simple_cost,
    OptimizerType,
    Supercharger
)

if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Burlington_WA"
    destination = "Marathon_FL"

    # Create the supercharger using Dijkstra's algorithm + optimization
    supercharger = Supercharger(
        cost_f=dijkstras_simple_cost,
        optim_type=OptimizerType.NLOPT)

    # Set the vehicle's speed and max range
    supercharger.max_range = 320
    supercharger.speed = 105

    # Plan the route with Dijkstra's algorithm
    start = perf_counter()
    result = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    print(f"Route Planning time: {(stop - start) * 1e3:.2f} ms")
    print(f"Dijkstra's + Post Optimization (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Now plan the same route using the optimized cost function
    supercharger.set_planner(cost_f=optimized_cost)
    supercharger.set_optimizer(type=OptimizerType.NONE)
    start = perf_counter()
    result = supercharger.plan_route(origin, destination)
    stop = perf_counter()
    print(f"\nRoute Planning time: {(stop - start) * 1e3:.2f} ms")
    print(f"Dijkstra's + Optimized Cost (Cost: {result.cost:.4f} hrs)")
    print(result)
