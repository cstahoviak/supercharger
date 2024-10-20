"""
Unit tests for the Optimizer module.
"""
from functools import partial

import numpy as np
import pytest
from scipy import optimize

from supercharger.optimize.optimizer import (
    NonlinearOptimizer,
    objective,
    objective_grad
)
from supercharger.optimize.constraints import (
    get_arrival_range,
    ineq_constraint,
    ineq_constraint_grad,
    eq_constraint,
    eq_constraint_grad
)

from supercharger.pysupercharger import (
    AlgorithmType,
    distance,
    Supercharger
)


@pytest.fixture
def planner_result():
    """
    The test_optimizer test fixture generates a PlannerResult using Dijkstra's
    algorithm.
    """
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the supercharger app using Dijkstra's algorithm
    supercharger = Supercharger(algo_type=AlgorithmType.DIJKSTRAS)

    # Set the vehicle's speed and max range
    supercharger.max_range = 320
    supercharger.speed = 105

    # Plan the route with Dijkstra's algorithm
    return supercharger.plan_route(origin, destination)


@pytest.fixture
def constraint_data(planner_result):
    """
    Creates the constraint data.
    """
    return NonlinearOptimizer.create_constraint_data(planner_result)


@pytest.fixture
def eval_point(planner_result):
    """
    The state vector is the vector of charging durations for all nodes not
    including the first and last nodes
    """
    return np.array([node.duration for node in planner_result.route[1:-1]])


def test_get_arrival_range(planner_result, constraint_data):
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
    func = partial(get_arrival_range, data=constraint_data)
    arrival_ranges = func(durations[:-1])

    # For each node in the solution obtained by Dijkstra's, we expect the
    # arrival range to be zero.
    np.testing.assert_allclose(actual=arrival_ranges,
                               desired=np.zeros_like(arrival_ranges),
                               atol=1e-9)


def test_cost_fcn_gradient(planner_result, eval_point):
    """
    Validates the analytically derived cost function gradient against a
    (forward) finite-difference approximation of the gradient.
    """
    residual = optimize.check_grad(objective, objective_grad, eval_point)
    np.testing.assert_equal(actual=residual, desired=np.zeros_like(residual))


def test_ineq_constraint_fcn_grad(planner_result, constraint_data, eval_point):
    """
    Validates the analytically derived inequality constraint function gradient
    against a (forward) finite-difference approximation of the gradient.
    """
    residual = optimize.check_grad(
        ineq_constraint, ineq_constraint_grad, eval_point, constraint_data)
    np.testing.assert_equal(actual=residual, desired=np.zeros_like(residual))


def test_eq_constraint_fcn_grad(planner_result, constraint_data, eval_point):
    """
    Validates the analytically derived equality constraint function gradient
    against a (forward) finite-difference approximation of the gradient.
    """
    residual = optimize.check_grad(
        eq_constraint, eq_constraint_grad, eval_point, constraint_data)
    np.testing.assert_equal(actual=residual, desired=np.zeros_like(residual))


