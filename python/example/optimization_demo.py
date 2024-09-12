"""
A demonstration of the Python route optimizer.
"""
from time import perf_counter

import numpy as np
import plotly.graph_objs as go

from supercharger.optimizer import NonlinearOptimizer

from supercharger.pysupercharger import (
    AlgorithmType,
    PlannerResult,
    RoutePlanner
)

# dijkstras_color = 'rgb(246, 207, 113)'
# nl_color = 'rgb(102, 197, 204)'

dijkstras_color = 'rgb(55, 83, 109)'
nl_color = 'rgb(26, 118, 255)'

def plot_charging_durations(baseline: PlannerResult, optimized: PlannerResult):
    """
    Args:
        baseline: The baseline PlannerResult.
        optimized: The optimized PlannerResult
    """
    labels = [f'{node.name}<br>({node.charger.rate:.0f} km/hr)' for node in
              baseline.route]
    durations = [node.duration for node in baseline.route]
    optimized_durations = [node.duration for node in optimized.route]

    fig = go.Figure()
    fig.add_trace(go.Bar(
        x=labels,
        y=durations,
        name="Dijkstra's Algorithm",
        marker_color=dijkstras_color,
        text=[f'{duration:.4f}' for duration in durations],
        textposition='outside',
    ))
    fig.add_trace(go.Bar(
        x=labels,
        y=optimized_durations,
        name='Nonlinear Optimization',
        marker_color=nl_color,
        text=[f'{duration:.4f}' for duration in optimized_durations],
        textposition='outside'
    ))

    # Here we modify the tickangle of the xaxis, resulting in rotated labels.
    fig.update_layout(
        barmode='group',
        xaxis={'title': 'Node'},
        yaxis={'title': 'Charging duration [hrs]'},
        # xaxis_tickangle=-45,
        legend={'yanchor': 'top',
                'y': 0.99,
                'xanchor': 'left',
                'x': 0.01}
    )
    fig.show()

def plot_ranges(baseline: PlannerResult, optimized: PlannerResult):
    """
    Args:
        baseline: The baseline PlannerResult.
        optimized: The optimized PlannerResult
    """
    labels = [f'{node.name}<br>({node.charger.rate:.0f} km/hr)' for node in
              baseline.route]
    arrival_rng = [node.arrival_range for node in baseline.route]
    departure_rng = [node.departure_range for node in baseline.route]
    opt_arrival_rng = [node.arrival_range for node in optimized.route]
    opt_departure_rng = [node.departure_range for node in optimized.route]

    fig = go.Figure()

    # Plot the arrival ranges for the baseline and optimized results.
    fig.add_trace(go.Bar(
        x=labels,
        y=arrival_rng,
        name="Arrival Range (Dijkstra's)",
        marker_color=dijkstras_color,
        marker_pattern_shape='/',
        opacity=0.5,
        text=[f'{val:.1f}' for val in arrival_rng],
        textposition='outside'
    ))
    fig.add_trace(go.Bar(
        x=labels,
        y=opt_arrival_rng,
        name='Arrival Range (Nonlinear Optimization)',
        marker_color=nl_color,
        text=[f'{val:.1f}' for val in opt_arrival_rng],
        textposition='outside'
    ))

    # Add a filler bar
    fig.add_trace(go.Bar(
        x=labels,
        y=np.full_like(labels, np.nan),
        width=0.1,
        showlegend=False
    ))

    # Plot the departure ranges for the baseline and optimized results.
    fig.add_trace(go.Bar(
        x=labels,
        y=departure_rng,
        name="Departure Range (Dijkstra's)",
        marker_color=dijkstras_color,
        marker_pattern_shape='/',
        opacity=0.5,
        text=[f'{val:.1f}' for val in departure_rng],
        textposition='outside'
    ))
    fig.add_trace(go.Bar(
        x=labels,
        y=opt_departure_rng,
        name='Departure Range (Nonlinear Optimization)',
        marker_color=nl_color,
        text=[f'{val:.1f}' for val in opt_departure_rng],
        textposition='outside'
    ))

    fig.update_layout(
        barmode='group',
        xaxis={'title': 'Node'},
        yaxis={'title': 'Range [km]'},
        legend={'yanchor': 'top',
                'y': 0.99,
                'xanchor': 'right',
                'x': 1.02}
    )
    fig.show()



if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Create the route planner using Dijkstra's algorithm
    planner = RoutePlanner(algo_type=AlgorithmType.DIJKSTRAS)

    # Set the vehicle's speed and max range
    planner.max_range = 320
    planner.speed = 105

    # Plan the route with Dijkstra's algorithm
    start = perf_counter()
    result = planner.plan_route(origin, destination)
    stop = perf_counter()
    print(f"Route Planning time: {(stop - start) * 1e3:.2f} ms")
    print(f"Dijkstra's Planner Final Route (Cost: {result.cost:.4f} hrs)")
    print(result)

    # Optimize the route
    optimizer = NonlinearOptimizer()
    start = perf_counter()
    optimized = optimizer.optimize(result)
    stop = perf_counter()
    print(f"\nOptimization time: {(stop - start) * 1e3:.2f} ms")
    print(f"Optimized Final Route (Cost: {optimized.cost:.4f} hrs)")
    print(optimized)

    # Plot the results
    plot_charging_durations(result, optimized)
    plot_ranges(result, optimized)
