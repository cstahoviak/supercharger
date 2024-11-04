"""
Plotly plotting utility functions.
"""
from typing import List

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go

from pysupercharger import PlannerResult

DIJKSTRAS_COLOR = 'rgb(55, 83, 109)'
NL_COLOR = 'rgb(26, 118, 255)'

mpl.rcParams['text.usetex'] = True
# mpl.rcParams['font.family'] = 'serif'
# mpl.rcParams['font.serif'] = ['Computer Modern']


def plot_durations(baseline: PlannerResult, optimized: PlannerResult):
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


def plot_stacked_durations(baseline: PlannerResult, optimized: PlannerResult):
    """
    Args:
        baseline: The baseline PlannerResult.
        optimized: The optimized PlannerResult
    """
    labels = [f'{node.name}\n({node.charger.rate:.0f} km/hr)' for node in
              baseline.route]
    x = np.arange(len(labels))

    baseline_durations = np.array([node.duration for node in baseline.route])
    optimized_durations = np.array([node.duration for node in optimized.route])

    n = len(baseline.route)
    baseline_running_total = np.array([baseline_durations[:idx].sum() for
                                       idx in range(n)])
    optimized_running_total = np.array([optimized_durations[:idx].sum() for
                                       idx in range(n)])

    fig, ax = plt.subplots(figsize=(12,8))
    width = 0.35
    alpha = 0.4
    bottom = np.zeros(n)

    bars = []
    bars.append(ax.bar(
        x=x - width/2,
        height=baseline_running_total,
        width=width,
        bottom=bottom,
        label="Dijkstra's Algorithm Charging Profile",
        color="tab:gray",
        alpha=alpha)
    )
    bars.append(ax.bar(
        x=x - width/2,
        height=baseline_durations,
        width=width,
        bottom=baseline_running_total,
        color="tab:gray")
    )

    bars.append(ax.bar(
        x=x + width/2,
        height=optimized_running_total,
        width=width,
        bottom=bottom,
        label="Optimized Charging Profile",
        color="tab:blue",
        alpha=alpha)
    )
    bars.append(ax.bar(
        x=x + width/2,
        height=optimized_durations,
        width=width,
        bottom=optimized_running_total,
        color="tab:blue")
    )

    # Add centered labels to all bars
    bar_labels = []
    get_bar_labels = lambda arr: [f'{val:.4f}' if val else '' for val in arr]
    bar_labels.append(get_bar_labels(baseline_running_total))
    bar_labels.append(get_bar_labels(baseline_durations))
    bar_labels.append(get_bar_labels(optimized_running_total))
    bar_labels.append(get_bar_labels(optimized_durations))
    for label, bar in zip(bar_labels, bars):
        ax.bar_label(bar, labels=label, fmt="%.4f", label_type='center')

    # Add horizontal lines for max values
    plt.axhline(y=baseline_running_total[-1],
                color="tab:gray",
                linestyle='--',
                label="Dijkstra's Algorithm Total Charge Time")
    plt.axhline(y=optimized_running_total[-1],
                color="tab:blue",
                linestyle='--',
                label="Optimized Total Charge Time")

    ax.set_title("Total Charging Duration Improvement via Constrained "
                 "Optimization")
    ax.set_ylabel('Charging duration [hrs]')
    ax.set_xticks(x, labels)
    ax.set_ylim(0, int(baseline_running_total[-1] + 1))

    # Add ytick labels
    y_ticks = np.append(ax.get_yticks(),
        [baseline_running_total[-1], optimized_running_total[-1]])
    ax.set_yticks(y_ticks)

    # Reordering the labels
    handles, labels = plt.gca().get_legend_handles_labels()
    order = [2, 3, 0, 1]
    legend = plt.legend([handles[i] for i in order], [labels[i] for i in order])
    legend.set_draggable(True)

    plt.show()


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


def plot_cost_vs_distance(distances: np.ndarray,
                          costs: np.ndarray,
                          labels: List[str]):
    """
    """
    idxs = np.argsort(distances)
    d = distances[idxs]

    # Plot route costs as a function of distance
    fig, ax = plt.subplots(figsize=(12, 8))
    for label, cost in zip(labels, costs.T):
        # ax.scatter(
        #     x=distances,
        #     y=cost,
        #     s=10,
        #     facecolors='None',
        #     label=label,
        #     alpha=0.75
        # )

        ax.plot(d, cost[idxs], label=label, alpha=0.75)

    ax.set_xlabel('Route distance [km]')
    ax.set_ylabel('Route cost [hrs]')
    ax.legend()
    plt.show()