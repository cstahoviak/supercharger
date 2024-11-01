"""
Replicates the cpp/app/supercharger.cpp application.

The output of this application is a single string in the format required by the
'checker_*' applications used for route validation. The required output
format looks like:

    <origin>, <node-1>, <charging-time-1>, <node-2>, <charging-time-2>, ...
    <node-n>, <charging-time-n>, <destination>
"""
from supercharger.pysupercharger import (
    dijkstras_simple_cost,
    OptimizerType,
    Supercharger
)

if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the supercharger using Dijkstra's algorithm
    supercharger = Supercharger(
        cost_f=dijkstras_simple_cost,
        optim_type=OptimizerType.NLOPT)

    # Set the vehicle's speed and max range
    supercharger.max_range = 320
    supercharger.speed = 105

    # Plan the route with Dijkstra's algorithm
    result = supercharger.plan_route(origin, destination)
    print(f"Dijkstra's Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)