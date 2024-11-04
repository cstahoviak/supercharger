"""
Demos the supercharger application for the route shown in the README.
"""
from pysupercharger import (
    AlgorithmType,
    DijkstrasPlanner,
    dijkstras_simple_cost,
    NaiveCostType,
    NaivePlanner
)

if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Define some vehicle constants
    max_range = 320
    speed = 105

    # Create the "naive" planner
    naive_planner = NaivePlanner(
        cost_type=NaiveCostType.MINIMIZE_TIME_REMAINING)

    # Plan the route with the "naive" route planner
    result = naive_planner.plan_route(origin, destination, max_range, speed)
    print(f"Naive Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Plan the route with Dijkstra's algorithm
    dijkstras_planner = DijkstrasPlanner(cost_f=dijkstras_simple_cost)
    result2 = dijkstras_planner.plan_route(origin, destination, max_range, speed)
    print(f"\nDijkstra's Planner Final Route (Cost: {result2.cost:.4f} hrs)")
    print(result2)
