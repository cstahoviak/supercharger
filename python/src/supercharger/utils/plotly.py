"""
Plotly plotting utility functions.
"""
import numpy as np
import plotly.graph_objs as go

from supercharger.pysupercharger import PlannerResult

DIJKSTRAS_COLOR = 'rgb(55, 83, 109)'
NL_COLOR = 'rgb(26, 118, 255)'


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

    # Plot the charging durations for Dijkstra's algorithm
    fig.add_trace(go.Bar(
        x=labels,
        y=durations,
        name="Dijkstra's Algorithm",
        marker_color=DIJKSTRAS_COLOR,
        text=[f'{duration:.4f}' for duration in durations],
        textposition='outside')
    )

    # Plot the charging durations for the nonlinear optimization solution
    fig.add_trace(go.Bar(
        x=labels,
        y=optimized_durations,
        name="Nonlinear Optimization",
        marker_color=NL_COLOR,
        text=[f'{duration:.4f}' for duration in optimized_durations],
        textposition='outside')
    )

    # Update the figure layout
    fig.update_layout(
        barmode='group',
        xaxis={'title': 'Node'},
        yaxis={'title': 'Charging duration [hrs]'},
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
        marker_color=DIJKSTRAS_COLOR,
        marker_pattern_shape='/',
        opacity=0.5,
        text=[f'{val:.1f}' for val in arrival_rng],
        textposition='outside'
    ))
    fig.add_trace(go.Bar(
        x=labels,
        y=opt_arrival_rng,
        name='Arrival Range (Nonlinear Optimization)',
        marker_color=NL_COLOR,
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
        marker_color=DIJKSTRAS_COLOR,
        marker_pattern_shape='/',
        opacity=0.5,
        text=[f'{val:.1f}' for val in departure_rng],
        textposition='outside'
    ))
    fig.add_trace(go.Bar(
        x=labels,
        y=opt_departure_rng,
        name='Departure Range (Nonlinear Optimization)',
        marker_color=NL_COLOR,
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