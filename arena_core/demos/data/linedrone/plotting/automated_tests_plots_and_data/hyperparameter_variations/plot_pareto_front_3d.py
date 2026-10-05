"""3D visualization of the estimated (reference) Pareto front.

Reads one or more Pareto front files written by testbench_node (*_pareto_front.csv, one row per non-dominated solution
of every run), computes the non-dominated set of their union (the estimated Pareto front) and plots it in the
objective space (time, safety and energy costs). The fronts of the individual runs are shown in light gray and the
best observed solution of every objective is highlighted.

Usage:
    python3 plot_pareto_front_3d.py [pareto front files or folders ...] [--projections] [--color-by energy]
                                    [--view 25 -60] [--output <file prefix>] [--no-runs]
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
from matplotlib import pyplot as plt

from plot_computation_analysis import PAPER_STYLE


HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_FRONT = os.path.join(HERE, 'CL_map_3', 'reference_front', 'report_3d_reference_pareto_front.csv')

# Objective: (column of the Pareto front file, axis label, scale applied for display)
OBJECTIVES = {
    'time': ('Time cost', 'Time cost (s)', 1.0),
    'safety': ('Security cost', 'Safety cost', 1.0),
    'energy': ('Energy cost', 'Energy cost (MJ)', 1.0e-6),
}
MAX_RUN_POINTS = 20000  # Points of the individual runs drawn in the background (random subset, for speed)
ZOOM_MARGIN = 0.15  # Margin around the estimated front when zooming on it, fraction of its range


def find_front_files(paths):
    files = []
    for path in paths:
        if os.path.isdir(path):
            files += sorted(glob.glob(os.path.join(path, '*_pareto_front.csv')))
        else:
            files.append(path)
    return files


def load_fronts(files):
    """Objective values of all the front points (n, 3), the number of runs and the (file, run Id) of every point."""
    points, sources, nb_of_runs = [], [], 0
    for path in files:
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        if rows and OBJECTIVES['time'][0] not in rows[0]:
            sys.exit('{} is not a Pareto front file written by testbench_node'.format(path))
        points += [[float(row[column]) for column, _, _ in OBJECTIVES.values()] for row in rows]
        sources += [(os.path.basename(path), row['Id']) for row in rows]
        nb_of_runs += len(set(row['Id'] for row in rows))
        print('{}: {} points, {} runs'.format(os.path.basename(path), len(rows), len(set(row['Id'] for row in rows))))
    return np.array(points, dtype=float).reshape(-1, len(OBJECTIVES)), nb_of_runs, sources


def non_dominated(points):
    """Non-dominated points (minimization).

    After a lexicographic sort, a point can only be dominated by points placed before it, so a single pass comparing
    every point to the front kept so far is enough (fast with many dominated points).
    """
    points = np.unique(points, axis=0)  # Sorted lexicographically, duplicates removed
    front = np.empty((0, points.shape[1]))
    for point in points:
        if len(front) and np.any(np.all(front <= point, axis=1) & np.any(front < point, axis=1)):
            continue
        front = np.vstack([front, point])
    return front


def zoom_limits(front):
    """Axes limits around the estimated front (the fronts of the runs extend much further)."""
    low, high = front.min(axis=0), front.max(axis=0)
    margin = ZOOM_MARGIN * np.where(high > low, high - low, np.abs(high) + 1e-9)
    return low - margin, high + margin


def scaled(points):
    return points * np.array([scale for _, _, scale in OBJECTIVES.values()])


def plot_3d(ax, front, run_points, color_by, show_runs):
    names = list(OBJECTIVES)
    labels = [label for _, label, _ in OBJECTIVES.values()]
    front_s = scaled(front)

    if show_runs and len(run_points):
        runs_s = scaled(run_points)
        ax.scatter(runs_s[:, 0], runs_s[:, 1], runs_s[:, 2], s=1, color='0.75', alpha=0.25, depthshade=False,
                   label='Fronts of the runs', rasterized=True)

    c = names.index(color_by)
    scatter = ax.scatter(front_s[:, 0], front_s[:, 1], front_s[:, 2], s=6, c=front_s[:, c], cmap='viridis',
                         depthshade=False, label='Estimated Pareto front')

    # Best observed solution of every objective
    for i, (name, marker) in enumerate(zip(names, ['*', 'P', 'D'])):
        best = front_s[np.argmin(front_s[:, i])]
        ax.scatter(*best, s=70, marker=marker, color='red', edgecolor='black', linewidth=0.6, depthshade=False,
                   label='Best {}'.format(name), zorder=10)

    ax.set_xlabel(labels[0], labelpad=4)
    ax.set_ylabel(labels[1], labelpad=4)
    ax.set_zlabel(labels[2], labelpad=4)
    ax.tick_params(pad=0)
    return scatter, labels[c]


def plot_projection(ax, front, run_points, i, j, show_runs):
    labels = [label for _, label, _ in OBJECTIVES.values()]
    front_s = scaled(front)
    if show_runs and len(run_points):
        runs_s = scaled(run_points)
        ax.scatter(runs_s[:, i], runs_s[:, j], s=1, color='0.75', alpha=0.25, rasterized=True)
    # Points of the front that are also non-dominated in this 2D projection, joined by a line
    projection = non_dominated(front[:, [i, j]])
    projection_s = scaled_pair(projection, i, j)
    ax.scatter(front_s[:, i], front_s[:, j], s=4, color='tab:blue')
    ax.plot(projection_s[:, 0], projection_s[:, 1], color='tab:red', linewidth=1.0)
    ax.set_xlabel(labels[i])
    ax.set_ylabel(labels[j])
    ax.grid(True, linewidth=0.4, alpha=0.4)


def scaled_pair(points, i, j):
    scales = [scale for _, _, scale in OBJECTIVES.values()]
    return points * np.array([scales[i], scales[j]])


def save_front(path, front, points, sources):
    """Writes the estimated Pareto front, with the file and run Id that found every solution."""
    source_of = {}
    for point, source in zip(map(tuple, points), sources):
        source_of.setdefault(point, source)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([column for column, _, _ in OBJECTIVES.values()] + ['File', 'Id'])
        for point in front:
            writer.writerow(['{:.6f}'.format(v) for v in point] + list(source_of[tuple(point)]))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('fronts', nargs='*', default=[DEFAULT_FRONT], help='Pareto front files or folders')
    parser.add_argument('--projections', action='store_true', help='Add the three 2D projections of the front')
    parser.add_argument('--color-by', choices=list(OBJECTIVES), default='energy', help='Objective giving the color')
    parser.add_argument('--view', nargs=2, type=float, default=[25.0, -60.0], metavar=('ELEV', 'AZIM'),
                        help='3D view angles in degrees')
    parser.add_argument('--no-runs', action='store_true', help='Do not draw the fronts of the individual runs')
    parser.add_argument('--full-range', action='store_true',
                        help='Show the whole range of the fronts of the runs instead of zooming on the estimated front')
    parser.add_argument('--output', help='Prefix of the figure files (default: next to the first front file)')
    parser.add_argument('--dpi', type=int, default=400)
    args = parser.parse_args()

    files = find_front_files(args.fronts)
    if not files:
        sys.exit('No Pareto front file found')
    points, nb_of_runs, sources = load_fronts(files)

    front = non_dominated(points)
    print('Estimated Pareto front: {} non-dominated points out of {} points of {} runs'.format(
        len(front), len(points), nb_of_runs))
    for i, (name, (_, label, _)) in enumerate(OBJECTIVES.items()):
        best = front[np.argmin(front[:, i])]
        print('  {:<7} range [{:.6g}, {:.6g}], best observed solution (time, safety, energy) = ({:.6g}, {:.6g}, {:.6g})'.format(
            name, front[:, i].min(), front[:, i].max(), *best))

    # By default only the points of the runs around the estimated front are drawn, the axes zoom on the front
    run_points = points
    limits = None
    if not args.full_range:
        limits = zoom_limits(front)
        run_points = points[np.all((points >= limits[0]) & (points <= limits[1]), axis=1)]
        print('Zoom on the estimated front: {} of the {} points of the runs are in the view'.format(len(run_points), len(points)))
    rng = np.random.default_rng(0)
    if len(run_points) > MAX_RUN_POINTS:
        run_points = run_points[rng.choice(len(run_points), MAX_RUN_POINTS, replace=False)]

    plt.rcParams.update(PAPER_STYLE)
    if args.projections:
        fig = plt.figure(figsize=(7.16, 5.6))
        ax3d = fig.add_subplot(2, 2, 1, projection='3d')
        projection_axes = [fig.add_subplot(2, 2, k) for k in (2, 3, 4)]
    else:
        fig = plt.figure(figsize=(3.5, 3.8))
        ax3d = fig.add_subplot(1, 1, 1, projection='3d')
        projection_axes = []

    scatter, color_label = plot_3d(ax3d, front, run_points, args.color_by, not args.no_runs)
    ax3d.view_init(elev=args.view[0], azim=args.view[1])
    # Shrink the 3D box in its axes so the axis labels are not cut (box zoom from matplotlib 3.6, camera distance before)
    try:
        ax3d.set_box_aspect(None, zoom=0.85)
    except TypeError:
        ax3d.dist = 11.5
    if limits is not None:
        low, high = scaled(limits[0][None, :])[0], scaled(limits[1][None, :])[0]
        ax3d.set_xlim(low[0], high[0])
        ax3d.set_ylim(low[1], high[1])
        ax3d.set_zlim(low[2], high[2])
    # Colorbar below the 3D axes so it doesn't hide the z label
    colorbar = fig.colorbar(scatter, ax=ax3d, orientation='horizontal', shrink=0.7, pad=0.1, aspect=30)
    colorbar.set_label(color_label)
    ax3d.legend(loc='upper left', bbox_to_anchor=(0.0, 1.08), fontsize=6.5, frameon=False, markerscale=0.8, ncol=2,
                columnspacing=0.8, handletextpad=0.3)

    for ax, (i, j) in zip(projection_axes, [(0, 1), (0, 2), (1, 2)]):
        plot_projection(ax, front, run_points, i, j, not args.no_runs)
        if limits is not None:
            scales = [scale for _, _, scale in OBJECTIVES.values()]
            ax.set_xlim(limits[0][i] * scales[i], limits[1][i] * scales[i])
            ax.set_ylim(limits[0][j] * scales[j], limits[1][j] * scales[j])

    fig.tight_layout(pad=0.4)
    prefix = args.output or os.path.join(os.path.dirname(os.path.abspath(files[0])), 'pareto_front_3d')
    save_front(prefix + '_estimated_front.csv', front, points, sources)
    nb_of_contributing_runs = len(set(map(tuple, (s for p, s in zip(map(tuple, points), sources)
                                                  if p in set(map(tuple, front))))))
    print('Estimated Pareto front written to {}_estimated_front.csv ({} runs contribute to it)'.format(
        prefix, nb_of_contributing_runs))
    for extension in ['pdf', 'png']:
        fig.savefig('{}.{}'.format(prefix, extension), dpi=args.dpi)
    print('Figure written to {}.pdf and {}.png'.format(prefix, prefix))
    plt.show()


if __name__ == '__main__':
    main()
