"""

"""
from dataclasses import dataclass, field, InitVar
from typing import Sequence

import numpy as np


@dataclass
class ConstraintData:
    """
    distances: (n-2,) The known distances between nodes, e.g. distances[0] is
        the distance from the second node to the third node.
    rates: (n-2,) The known charging rates at each node for nodes [2, n-1].
    init_arrival_rng: The known arrival range at node two (the node following
        the origin).

    The A matrix encodes the charging rates at each node and looks like:

    [[r1,  0,  0, ..., 0],
     [r1, r2,  0, ..., 0],
     [r1, r2, r3, ..., 0],
     ...
     [r1, r2, r3, ..., r_(n-1)]]

     where r1 is the charging rate at the first node (the node following the
     origin), and r_(n-1) is the charging rate at the second to last node.

     A_ineq = A[:-1]
     A_eq = A[-1]
    """
    distances: np.ndarray
    rates: InitVar[np.ndarray]
    init_arrival_range: float

    n: int = field(init=False)
    A: np.ndarray = field(init=False)
    A_ineq: np.ndarray = field(init=False)
    A_eq: np.ndarray = field(init=False)

    def __post_init__(self, rates):
        # Define the A matrix
        self.n = len(rates)
        self.A = np.tril(np.tile(rates, (self.n, 1)))

        # Define the A matrix for the inequality constraint
        self.A_ineq = self.A[:-1]

        # Define the A matrix for the equality constraint
        self.A_eq = self.A[-1]


def get_arrival_range(
        durations: Sequence[float],
        data: ConstraintData) -> np.ndarray:
    """
    The arrival range at each node [2, n] is computed as:

    arrival_range(x) = A.dot() - L.dot(d) + a_1

    where A is a lower triangular matrix composed of the charging rates at each
    node, x is the vector of charging durations, L is a unit lower triangular
    matrix, d is a vector of distances between nodes and a_1 is the arrival
    range at the first node (the node following the origin node).

    Args:
        durations: (n-2,) A list of charging durations for all nodes but the
            first and last node, [2, n-1].
        data: A ConstraintData instance.

    Returns: (n-2,) The arrival range at each node from the third node onward.
    """
    return data.A.dot(durations) - np.tri(len(durations)).dot(data.distances) \
        + data.init_arrival_range


def ineq_constraint(x: Sequence[float], constr_data: ConstraintData) -> \
        np.ndarray:
    """
    The inequality constraint function. Applies to all nodes between the second
    node and the second to last node, [2, n-1].
    """
    return get_arrival_range(x, constr_data)[:-1]


def ineq_constraint_grad(x: Sequence[float], constr_data: ConstraintData) -> \
        np.ndarray:
    """
    The inequality constraint function gradient, an mxn matrix where n is the
    dimension of the vector x (and the number of nodes whose charging durations
    are being optimized), and m = n-1 (the number of nodes that the inequality
    constraint applies to).
    """
    # Validation of C++-style lower-triangular matrix generation
    # n = len(x)
    # m = n - 1
    # grad = np.zeros(m * n)
    # for idx_m in range(m):
    #     for idx_n in range(n):
    #         if idx_n <= idx_m:
    #             grad[idx_m * n + idx_n] = constr_data.rates[idx_n]

    return constr_data.A_ineq


def eq_constraint(x: Sequence[float], constr_data: ConstraintData) -> \
        np.ndarray:
    """
    The equality constraint applies to the last node - a time optimal route will
    have an arrival range of zero at the final node.
    """
    return get_arrival_range(x, constr_data)[-1]


def eq_constraint_grad(x: Sequence[float], constr_data: ConstraintData) -> \
        np.ndarray:
    """
    The equality constraint function gradient, an nx1 vector where n is the
    dimension of the vector x (and the number of nodes whose charging durations
    are being optimized).
    """
    return constr_data.A_eq