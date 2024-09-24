"""
Unit tests for the Optimizer module.
"""
from functools import partial

import numpy as np
import pytest
from scipy import optimize

from supercharger.optimizer import get_arrival_range, cost_fcn, cost_fcn_grad

from supercharger.pysupercharger import (
    AlgorithmType,
    distance,
    RoutePlanner
)

@pytest.fixture
def planner_result():
    """
    The test_optimizer test fixture.
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
    return result


def test_nonlinear_constraint(planner_result):
    """
    Verifies that the nonlinear constraint function is correct.
    """
    # Define some arrays for route optimization
    distances = []
    durations = []
    rates = []
    for idx, node in enumerate(planner_result.route[1:]):
        distances.append(distance(node, planner_result.route[idx]))
        durations.append(node.duration)
        rates.append(node.charger.rate)

    # Compute the arrival range at each node not including the origin.
    func = partial(get_arrival_range,
                   rates=rates[:-1],
                   distances=distances[1:],
                   init_arrival_rng=planner_result.route[1].arrival_range)
    arrival_ranges = func(durations[:-1])

    # For each node in the solution obtained by Dijkstra's, we expect the
    # arrival range to be zero.
    np.testing.assert_equal(actual=arrival_ranges,
                            desired=np.zeros_like(arrival_ranges))


def test_cost_fcn_gradient(planner_result):
    """
    Validates the analytically derived cost function gradient against a
    (forward) finite-difference approximation of the gradient.
    """
    # Get the durations to use as the evaluation point
    durations = [node.duration for node in planner_result.route]

    residual = optimize.check_grad(cost_fcn, cost_fcn_grad, durations)
    np.testing.assert_equal(residual, np.zeros_like(residual))


def test_constraint_fcn_grad(planner_result):
    """
    Validates the analytically derived constraint function gradient against a
    (forward) finite-difference approximation of the gradient.
    """
    pass