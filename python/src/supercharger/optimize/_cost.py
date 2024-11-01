"""
An optimized cost function for Dijkstra's algorithm.
"""
from supercharger.optimize import (

)

from supercharger.pysupercharger import (
    Node
)


def optimized_cost(current: Node, neighbor: Node, speed: float) -> float:
    """

    Args:
        current: The current node.
        neighbor: The neighbor node.
        speed: [km/hr] The vehicle's constant velocity.

    Returns:
        The cost to arrive at the neighbor node via the current node.
    """
