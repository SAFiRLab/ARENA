"""Hypervolume of the front of every reference run compared to the reconstructed best (estimated) Pareto front.

The estimated Pareto front is the non-dominated set of the union of the fronts of all the reference runs
(reference_front.sh). For every run, in the order the runs were executed, the hypervolume of its own front is
computed and compared to the hypervolume of the estimated front (horizontal dashed line).
With --union, the hypervolume of the union of the fronts of the first k runs is also drawn: it reaches the dashed
line at the last run, its flattening shows that adding runs no longer improves the front.

All the hypervolumes use the same scaling and the same reference point. Every objective is divided by its value at
the nadir point of the estimated front (worst value of the objective on the front, no shift, no min-max
normalization), so the objectives are dimensionless ratios and 1 is the worst value of the estimated front:
    --ref-point 1.1   (default) 1.1 on every scaled objective, i.e. 1.1 times the nadir point of the estimated front
                      (standard choice), runs whose front is entirely outside this box get 0
    --ref-point worst 1.1 times the worst scaled value of all the fronts of the runs, every run gets a value

Usage:
    python3 plot_reference_convergence.py [reference report folders ...] [--ref-point 1.1|worst] [--union]
                                          [--output <file prefix>] [--size 3.5 2.6] [--dpi 400]
"""

import argparse
import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt

from compute_pareto_metrics import find_reports, hypervolume_3d, load_runs
from plot_computation_analysis import DEFAULT_REFERENCE, FIGURE_SIZE, LINE_WIDTH, PAPER_STYLE
from plot_pareto_front_3d import non_dominated


def union_hypervolumes(fronts, normalize, ref_point):
    """Hypervolume of the union of the fronts of the first k runs, for every k."""
    hypervolumes = []
    union = np.empty((0, 3))
    for front in fronts:
        union = non_dominated(np.vstack([union, front]))
        hypervolumes.append(hypervolume_3d(normalize(union), ref_point))
    return np.array(hypervolumes)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('reference', nargs='*', default=DEFAULT_REFERENCE, help='Report folders of the reference runs')
    parser.add_argument('--ref-point', default='1.1', help='HV reference point: a value on every normalized objective, or worst')
    parser.add_argument('--union', action='store_true', help='Also draw the hypervolume of the union of the first k fronts')
    parser.add_argument('--output', default=os.path.join(os.path.dirname(os.path.realpath(__file__)), 'reference_convergence'),
                        help='Prefix of the figure and CSV files')
    parser.add_argument('--size', nargs=2, type=float, default=list(FIGURE_SIZE), metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--dpi', type=int, default=400)
    args = parser.parse_args()

    runs = []
    for report, pareto in find_reports(args.reference):
        runs += load_runs(report, pareto, all_safe_solutions=True)
    # Runs in the order they were executed
    runs = sorted(runs, key=lambda r: (r['file'], r['id']))
    if not any(r['feasible'] and len(r['front']) for r in runs):
        sys.exit('No feasible reference run found')
    fronts = [non_dominated(r['front']) if r['feasible'] and len(r['front']) else np.empty((0, 3)) for r in runs]
    iterations = np.arange(1, len(runs) + 1)

    # Reconstructed best (estimated) Pareto front, the scaling of the objectives and the reference point of every
    # hypervolume: ratio to the nadir point of the estimated front, no shift
    estimated_front = non_dominated(np.vstack([f for f in fronts if len(f)]))
    nadir = estimated_front.max(axis=0)
    normalize = lambda f: f / nadir
    if args.ref_point == 'worst':
        worst = np.max(np.vstack([normalize(f) for f in fronts if len(f)]), axis=0)
        ref_point = 1.1 * np.maximum(worst, 1.0)
    else:
        ref_point = np.full(3, float(args.ref_point))

    estimated_hv = hypervolume_3d(normalize(estimated_front), ref_point)
    run_hv = np.array([hypervolume_3d(normalize(f), ref_point) if len(f) else 0.0 for f in fronts])

    print('{} reference runs, estimated Pareto front: {} non-dominated points'.format(len(runs), len(estimated_front)))
    print('Nadir point of the estimated front (scaling of the objectives): {}'.format(nadir))
    print('Hypervolume of the objectives divided by the nadir point, reference point {}:'.format(np.round(ref_point, 3)))
    print('  estimated Pareto front: {:.6f}'.format(estimated_hv))
    print('  runs: mean {:.6f}, std {:.6f}, median {:.6f}, max {:.6f} (run {}), {} runs with a null hypervolume'.format(
        run_hv.mean(), run_hv.std(ddof=1), np.median(run_hv), run_hv.max(), iterations[np.argmax(run_hv)],
        int(np.sum(run_hv == 0.0))))
    for fraction in [0.5, 0.8, 0.9]:
        print('  runs above {:.0f} % of the estimated front: {}'.format(
            100 * fraction, int(np.sum(run_hv >= fraction * estimated_hv))))

    union_hv = union_hypervolumes(fronts, normalize, ref_point) if args.union else None

    with open(args.output + '.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Iteration', 'File', 'Id', 'Hypervolume of the run front', 'Hypervolume of the union of the first runs',
                         'Hypervolume of the estimated front'])
        for i, run in enumerate(runs):
            writer.writerow([iterations[i], run['file'], run['id'], run_hv[i],
                             union_hv[i] if union_hv is not None else '', estimated_hv])

    plt.rcParams.update(PAPER_STYLE)
    fig, ax = plt.subplots(figsize=args.size)
    # Every run is an independent sample: one point per run
    ax.scatter(iterations, run_hv, s=6, color='tab:blue', linewidths=0, label='Front of the run', zorder=3)
    if union_hv is not None:
        ax.plot(iterations, union_hv, color='0.45', linewidth=LINE_WIDTH * 0.7, label='Union of the first runs')
    ax.axhline(estimated_hv, color='black', linewidth=1.0, linestyle='--', label='Estimated Pareto front')
    ax.set_xlim(0, len(runs) + 1)
    ax.set_ylim(bottom=0.0, top=1.05 * max(estimated_hv, run_hv.max()))
    ax.set_xlabel('Run')
    ax.set_ylabel('Hypervolume (objectives / nadir)')
    ax.grid(True, linewidth=0.4, alpha=0.4)
    # Above the axes, the runs fill the whole plot
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False, handlelength=1.8,
              columnspacing=1.0, handletextpad=0.4, borderaxespad=0.1, markerscale=2.0)
    fig.tight_layout(pad=0.3)
    for extension in ['pdf', 'png']:
        fig.savefig('{}.{}'.format(args.output, extension), dpi=args.dpi, bbox_inches='tight', pad_inches=0.02)
    print('Figure written to {0}.pdf and {0}.png, values written to {0}.csv'.format(args.output))
    plt.show()


if __name__ == '__main__':
    main()
