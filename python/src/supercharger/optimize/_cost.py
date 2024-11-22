"""
An optimized cost function for Dijkstra's algorithm.
"""
from supercharger.optimize._optimizer import NonlinearOptimizer

from pysupercharger import (
    dijkstras_simple_cost,
    DijkstrasNode,
    get_charge_time,
    get_arrival_range,
    NLOptimizer,
    PlannerResult
)


def optimized_cost(
        current: DijkstrasNode,
        neighbor: DijkstrasNode,
        max_range: float,
        speed: float) -> float:
    """
    An optimized cost function for Dijkstra's algorithm.

    Args:
        current: The current node.
        neighbor: The neighbor node.
        max_range: [km] The maximum range of the vehicle.
        speed: [km/hr] The vehicle's constant velocity.

    Returns:
        The cost to arrive at the neighbor node via the current node.
    """
    # Create the route as if the neighbor node were the destination.
    route = [neighbor]

    parent = current
    while parent is not None:
        # Add the parent node to the route and update the parent.
        route.append(parent)
        parent = parent.parent

    # Reverse the order of the nodes to get the correct route ordering
    route.reverse()

    # Can only optimize a route consisting of four or more nodes.
    if len(route) > 3:
        # Compute the charge time at each node
        for current_node, next_node in zip(route[:-1], route[1:]):
            # Compute the charge time at the current node to reach the neighbor.
            current_node.duration = get_charge_time(current_node, next_node)

            # Compute the arrival range at the neighbor node.
            next_node.arrival_range = get_arrival_range(current_node, next_node)

        # Create the result to be optimized
        current_result = PlannerResult(
            route=route,
            cost=0.0,
            max_range=max_range,
            speed=speed
        )

        # Optimize the route
        # optimizer = NonlinearOptimizer()
        optimizer = NLOptimizer()
        optimized = optimizer.optimize(current_result)
        cost = optimized.cost
    else:
        cost = dijkstras_simple_cost(current, neighbor, max_range, speed)

    return cost
