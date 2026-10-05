"""Plots the results of the RRT range variation test (scripts/linedrone/automated_tests/.../rrt_range_variation.sh).

Usage:
    python3 plot_rrt_range_variation.py [metric] [report folder]
        metric: feasibility, planning_time, time_cost, security_cost or energy_cost (default: planning_time)
        report folder: folder containing the report (default: CL_map_3/rrt_report1)

The costs are averaged over the feasible runs only, the costs of the infeasible runs are the max double.
"""

import os
import sys

import numpy as np
from matplotlib import pyplot as plt

from plot_hyperparameter_variations import PLOT_METRICS, find_report, open_csv


COST_COLUMNS = {'time_cost': 'Time cost', 'security_cost': 'Security cost', 'energy_cost': 'Energy cost'}


def extract_runs(header, data):
    """Returns one entry per planning (Id): RRT range, feasibility, planning time and chosen costs."""
    if 'RRT range' not in header:
        sys.exit('The report has no "RRT range" column, it has not been written by the RRT range variation test')

    runs = []
    old_id = None
    for row in data:
        if row[header.index('Id')] == old_id:
            continue
        old_id = row[header.index('Id')]

        run = {
            'rrt_range': float(row[header.index('RRT range')]),
            'feasibility': float(row[header.index('Feasible')]),
            'planning_time': float(row[header.index('Planing Time')]),
        }
        for metric, column in COST_COLUMNS.items():
            run[metric] = float(row[header.index(column)])
        runs.append(run)
    return runs


def get_data_from_rrt_range(runs, metric):
    """Mean and standard deviation of a metric for every RRT range."""
    rrt_ranges = sorted(set(r['rrt_range'] for r in runs))
    means, std_devs = [], []
    for rrt_range in rrt_ranges:
        group = [r for r in runs if r['rrt_range'] == rrt_range]
        if metric in COST_COLUMNS:
            group = [r for r in group if r['feasibility'] == 1.0]
        values = np.array([r[metric] for r in group], dtype=float)
        means.append(np.mean(values) if len(values) > 0 else np.nan)
        std_devs.append(np.std(values) if len(values) > 0 else np.nan)
    return np.array(rrt_ranges), np.array(means), np.array(std_devs)


if __name__ == '__main__':
    metric = sys.argv[1] if len(sys.argv) > 1 else 'planning_time'
    if metric not in PLOT_METRICS:
        sys.exit('Unknown metric "{}", choose one of: {}'.format(metric, ', '.join(PLOT_METRICS)))

    current_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'CL_map_3', 'report4')
    if len(sys.argv) > 2:
        current_path = os.path.abspath(sys.argv[2])

    header, data = open_csv(os.path.join(current_path, find_report(current_path)))
    runs = extract_runs(header, data)

    label, unit, scale = PLOT_METRICS[metric]
    rrt_ranges, out_mean, out_std_dev = get_data_from_rrt_range(runs, metric)
    out_mean *= scale
    out_std_dev *= scale

    # Plot the data with shaded standard deviation
    plt.plot(rrt_ranges, out_mean, label='RRT range variation', color='purple', marker='o', markersize=3)
    plt.fill_between(rrt_ranges, out_mean - out_std_dev, out_mean + out_std_dev, color='purple', alpha=0.2)

    # Labels and title
    plt.xlabel('Distance between the RRT nodes (m)')
    plt.ylabel('{} ({})'.format(label, unit) if unit else label)
    title = 'Mean and standard deviation of {}'.format(label.lower())
    if metric in COST_COLUMNS:
        title += ' (feasible runs)'
    plt.title(title)
    plt.legend()
    plt.show()
