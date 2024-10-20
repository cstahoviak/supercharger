"""
Demos the supercharger application for the route shown in the README.
"""
from supercharger.pysupercharger import (
    AlgorithmType,
    CostFunctionType,
    Supercharger
)

if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the supercharger app using the "naive" planning algorithm
    naive_planner = Supercharger(
        algo_type=AlgorithmType.NAIVE,
        cost_type=CostFunctionType.MINIMIZE_TIME_REMAINING
    )

    # Set the vehicle's speed and max range
    naive_planner.max_range = 320
    naive_planner.speed = 105

    # Get the charger network
    network = naive_planner.network
    print(network['Cadillac_MI'])

    # Plan the route with the "naive" route planner
    result = naive_planner.plan_route(origin, destination)
    print(f"Naive Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Create the supercharger app using Dijkstra's planning algorithm
    dijkstras_planner = Supercharger(algo_type=AlgorithmType.DIJKSTRAS)

    # Set the vehicle's speed and max range
    dijkstras_planner.max_range = 320
    dijkstras_planner.speed = 105

    # Plan the route with Dijkstra's algorithm
    result2 = dijkstras_planner.plan_route(origin, destination)
    print(f"Dijkstra's Planner Final Route (Cost: {result2.cost:.4f} hrs)")
    print(result2)

    # Optimize the route
    result3 = dijkstras_planner.optimize_route(result2)
    print(f"Optimized Final Route (Cost: {result3.cost:.4f} hrs)")
    print(result3)
