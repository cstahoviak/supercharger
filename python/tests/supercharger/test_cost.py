"""
Unit tests for Dijkstra's cost functions.
"""
import numpy as np

from supercharger.optimize import optimized_cost

from pysupercharger import (
    CostFunctionType,
    DijkstrasPlanner
)


def test_dijkstras_optimized_cost():
    """
    Validates the C++ implementation of the optimized Dijkstra's cost function
    against the python implementation.
    """
    # Define the route's endpoints
    origin = "Marathon_FL"
    destination = "Superior_MT"

    # Define some vehicle constants
    max_range = 320
    speed = 105

    # Create route planners
    py_planner = DijkstrasPlanner(cost_f=optimized_cost)
    cpp_planner = DijkstrasPlanner(
        cost_type=CostFunctionType.DIJKSTRAS_OPTIMIZED)

    # Plan the routes
    py_result = py_planner.plan_route(origin, destination, max_range, speed)
    cpp_result = cpp_planner.plan_route(origin, destination, max_range, speed)

    # Validate that all nodes along the route are the same
    for py_node, cpp_node in zip(py_result.route, cpp_result.route):
        np.testing.assert_string_equal(py_node.name, cpp_node.name)
        np.testing.assert_equal(py_node.cost, cpp_node.cost)

    # Validate that the total route cost is equal
    np.testing.assert_equal(py_result.cost, cpp_result.cost)