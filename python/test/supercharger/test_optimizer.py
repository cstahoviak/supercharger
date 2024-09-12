"""
Unit tests for the Optimizer module.
"""
from functools import partial

import numpy as np

from supercharger.optimizer import constraint

from supercharger.pysupercharger import (
    AlgorithmType,
    distance,
    RoutePlanner
)

def test_nonlinear_constraint():
    """
    Verifies that the nonlinear constraint function is correct.
    """
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the route planner using Dijkstra's algorithm
    planner = RoutePlanner(algo_type=AlgorithmType.DIJKSTRAS)

    # Set the vehicle's speed and max range
    planner.max_range = 320
    planner.speed = 105

    # Plan the route with Dijkstra's algorithm
    result = planner.plan_route(origin, destination)

    # Define some arrays for route optimization
    distances = []
    durations = []
    rates = []
    for idx, node in enumerate(result.route[1:]):
        distances.append(distance(node, result.route[idx]))
        durations.append(node.duration)
        rates.append(node.charger.rate)

    # Compute the arrival range at each node not including the origin.
    func = partial(constraint,
                   rates=rates[:-1],
                   distances=distances[1:],
                   init_arrival_rng=result.route[1].arrival_range)
    arrival_ranges = func(durations[:-1])

    # For each node in the solution obtained by Dijkstra's, we expect the
    # arrival range to be zero.
    np.testing.assert_allclose(actual=arrival_ranges,
                               desired=np.zeros_like(arrival_ranges),
                               atol=1e-12)
