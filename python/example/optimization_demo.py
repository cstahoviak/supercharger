"""
A demonstration of the Python route optimizer.
"""
from time import perf_counter

from supercharger.optimize import NonlinearOptimizer
from supercharger.utils.plotting import (
    plot_durations,
    plot_stacked_durations,
    plot_ranges
)

from pysupercharger import (
    CostFunctionType,
    DijkstrasPlanner
)

if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the supercharger using Dijkstra's algorithm
    planner = DijkstrasPlanner(cost_type=CostFunctionType.DIJKSTRAS_SIMPLE)

    # Set the vehicle's speed and max range
    max_range = 320
    speed = 105

    # Plan the route with Dijkstra's algorithm
    start = perf_counter()
    result = planner.plan(origin, destination, max_range, speed)
    stop = perf_counter()
    print(f"Route Planning time: {(stop - start) * 1e3:.2f} ms")
    print(f"Dijkstra's Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Optimize the route
    optimizer = NonlinearOptimizer()
    start = perf_counter()
    optimized = optimizer.optimize(result, method='SLSQP')
    stop = perf_counter()
    print(f"\nOptimization time: {(stop - start) * 1e3:.2f} ms")
    print(f"Optimized Final Route (Cost: {optimized.cost:.4f} hrs)")
    print(optimized)

    # Plot the results
    # plot_durations(result, optimized)
    plot_stacked_durations(result, optimized)
    # plot_ranges(result, optimized)

    # TODO: Create a stacked bar chart of charging durations that represents
    #  the total charging time up to that node.
