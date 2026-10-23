"""Risk sensitivity of the reference front and of ARENA in the same panels (merge of the two figures of
plot_risk_sensitivity.py --reference-front and --sweep-fronts --every-level).

Grid of 3 x 3 panels like plot_risk_sensitivity.py: one column per risk (wind, battery, localization) swept from 0 to 1 by
--step with the other risks null (or, with --sum-to-one, the other two risks at (1 - r) / 2 each so that the three risks
sum to 1 and decrease as the swept risk r grows; the communication risk stays null), one row per metric of the selected
solution: average obstacle clearance (mean distance of its trajectory to the inflated octomap, from the solutions exported
with their clearance, report_3d_*_solutions.csv next to the reports), path duration, energy:
    - Reference front (black dots): the voting replayed on the estimated reference front of the map
      (hyperparameter_variations/<map>_init2pct_res_clearance/reference_front/pareto_front_3d_estimated_front.csv), one
      solution per risk level
    - ARENA (line and band): the voting replayed on the front of every planning of <map>/default_params_rrt10 (ARENA with
      its default hyperparameters and an RRT range of 10 m, 50 plannings, see its description.txt), every front at every
      risk level, mean and standard deviation over the fronts per risk level
    - optimal lines: best solution of every objective of the reference front
The voting is RiskWeightedTchebycheffVoting of arena_core (eq11_exp, scaled influences, hazard risk transform, gain 2,
nominal coefficients 1/3).

Usage:
    python3 plot_risk_sensitivity_merged.py [--map CL_map_3] [--reference <csv>] [--sweeps <folder>] [--step 0.01]
        [--sum-to-one] [--output <prefix>] [--size 3.5 3.9] [--dpi 400]
"""

import argparse
import os

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from plot_risk_sensitivity import (HERE, OPTIMAL, PAPER_STYLE, RISKS, Y_MARGIN, load_clearances, load_reference_front,
                                   RISK_ORDER, load_sweep_fronts, rows_of_risk, selector)

REFERENCE_STYLE = {'color': '#0b0b0b', 'marker': 'o', 's': 2.5, 'linewidths': 0}
ARENA_COLOR = '#4a3aa7'  # OKLab dE >= 13 to the three optimal lines under normal vision and every CVD simulation
ARENA_BAND_ALPHA = 0.22

# Compact layout (single column, the paper is space limited): one-line labels, one-row legend, small fonts
# Rows: metric of the selected solution and its label (two lines for the clearance, the rows are short)
ROWS = [('clearance', 'Avg. obstacle\nclearance (m)'), ('duration', 'Duration (s)'), ('energy', 'Energy (kJ)')]
OPTIMAL_LABELS = {'Time optimal': 'Time opt.', 'Safety optimal': 'Safety opt.', 'Energy optimal': 'Energy opt.'}
COMPACT_STYLE = {'font.size': 7, 'axes.labelsize': 7, 'xtick.labelsize': 6.5, 'ytick.labelsize': 6.5,
                 'legend.fontsize': 6.5, 'axes.linewidth': 0.5, 'xtick.major.width': 0.5, 'ytick.major.width': 0.5,
                 'xtick.major.size': 2.0, 'ytick.major.size': 2.0, 'xtick.major.pad': 1.5, 'ytick.major.pad': 1.5,
                 'axes.labelpad': 1.5}

# Reference front of every map (estimated front of the reference runs with the 2 % A* and 2 % safe RRT initialization,
# every solution exported with its clearance)
REFERENCE_FRONTS = {'CL_map_3': os.path.join('CL_map_3_init2pct_res_clearance', 'reference_front'),
                    'seed_42': os.path.join('generated_maps', 'seed_42_init2pct_res_clearance', 'reference_front')}


def sweep_front(front, select, step=0.01, sum_to_one=False):
    """plot_risk_sensitivity.sweep_front with the risks not swept at 0, or at (1 - level) / 2 each with sum_to_one."""
    vote, metrics = front
    rows = []
    for column in RISK_ORDER:
        for level in np.round(np.arange(0.0, 1.0 + 1e-9, step), 6):
            risks = [(1.0 - level) / (len(RISK_ORDER) - 1) if sum_to_one else 0.0] * len(RISK_ORDER)
            risks[RISK_ORDER.index(column)] = level
            chosen = select(vote, *risks)
            row = {r: 0.0 for r in RISK_ORDER}
            row.update({column: level, 'column': column, 'solution': chosen})
            rows.append(dict(row, **metrics[chosen]))
    return rows


def mean_std(rows, risk, metric):
    """Levels of the risk, mean and standard deviation (over the fronts) of the metric per level."""
    levels = np.array(sorted({r[risk] for r in rows}))
    values = [np.array([r[metric] for r in rows if r[risk] == level]) for level in levels]
    values = [v[~np.isnan(v)] for v in values]  # A solution without exported clearance is left out
    return levels, np.array([v.mean() for v in values]), np.array([v.std(ddof=1) if len(v) > 1 else 0.0 for v in values])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', default='CL_map_3', choices=sorted(REFERENCE_FRONTS))
    parser.add_argument('--reference', help='Estimated reference front (default: REFERENCE_FRONTS of the map)')
    parser.add_argument('--sweeps', default='default_params_rrt10',
                        help='Folder of baselines/<map> holding the reports of the ARENA plannings and their exported '
                             'solutions (every report is read)')
    parser.add_argument('--step', type=float, default=0.01, help='Step of the risk levels')
    parser.add_argument('--sum-to-one', action='store_true',
                        help='The risks not swept compensate the swept risk r, (1 - r) / 2 each: the three risks sum to 1')
    parser.add_argument('--output', help='Prefix of the figure files (default: baselines/<map>/'
                                         'risk_sensitivity_reference_and_sweeps[_sum1]_scaled_hazard_g2_init2pct_avgObsClear)')
    parser.add_argument('--size', nargs=2, type=float, default=[3.5, 2.2], metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--dpi', type=int, default=400)
    args = parser.parse_args()

    select = selector('eq11_exp', gamma=2.0, influence='scaled', transform='hazard')
    reference_path = args.reference or os.path.join(HERE, '..', 'hyperparameter_variations', REFERENCE_FRONTS[args.map],
                                                    'pareto_front_3d_estimated_front.csv')
    output = args.output or os.path.join(HERE, args.map,
                                         'risk_sensitivity_reference_and_sweeps' + ('_sum1' if args.sum_to_one else '') +
                                         '_scaled_hazard_g2_init2pct_avgObsClear')

    # Reference front: one selected solution per risk level
    front = load_reference_front(reference_path, load_clearances([os.path.dirname(reference_path)]))
    reference = sweep_front(front, select, args.step, args.sum_to_one)
    print('Reference front {}: {} solutions, {} selected over the risk sweep'.format(
        reference_path, len(front[1]), len(set(r['solution'] for r in reference))))

    # ARENA: every front of the plannings replayed at every risk level
    folders = [os.path.join(HERE, args.map, args.sweeps)]
    plannings = load_sweep_fronts(folders, load_clearances(folders))
    arena = []
    for _, costs in plannings:
        metrics = [{'safety': c[1], 'duration': c[0], 'energy': c[2] * 1.0e-3, 'clearance': c[3]} for c in costs]
        arena += sweep_front((costs[:, :3], metrics), select, args.step, args.sum_to_one)
    print('ARENA: {} fronts of {}'.format(len(plannings), ', '.join(os.path.basename(f) for f in folders)))

    # Optimal solution of every objective on the reference front, one line each (they can coincide)
    optimal = []
    for _, objective, label, color, style in OPTIMAL:
        best = min(front[1], key=lambda m: m['costs'][objective])
        print('{} (reference front): clearance {:.2f} m, duration {:.1f} s, safety cost {:.3f}, energy {:.1f} kJ'.format(
            label, best['clearance'], best['duration'], best['safety'], best['energy']))
        optimal.append((OPTIMAL_LABELS[label], color, style, best))

    plt.rcParams.update(PAPER_STYLE)
    plt.rcParams.update(COMPACT_STYLE)
    fig, axes = plt.subplots(len(ROWS), len(RISKS), figsize=args.size, sharex=True, sharey='row')
    core_values = [[] for _ in ROWS]
    for col, (risk, risk_label) in enumerate(RISKS):
        reference_rows = rows_of_risk(reference, risk)
        arena_rows = rows_of_risk(arena, risk)
        for row, (metric, metric_label) in enumerate(ROWS):
            ax = axes[row, col]
            for label, color, style, best in optimal:
                ax.axhline(best[metric], color=color, linestyle=style, linewidth=0.8, zorder=3)
            levels, mean, std = mean_std(arena_rows, risk, metric)
            ax.fill_between(levels, mean - std, mean + std, color=ARENA_COLOR, alpha=ARENA_BAND_ALPHA, linewidth=0,
                            zorder=4)
            ax.plot(levels, mean, color=ARENA_COLOR, linewidth=0.8, zorder=5)
            ax.scatter([r[risk] for r in reference_rows], [r[metric] for r in reference_rows], zorder=6,
                       **REFERENCE_STYLE)
            core_values[row] += list(mean - std) + list(mean + std) + [r[metric] for r in reference_rows] + \
                [best[metric] for _, _, _, best in optimal]

            ax.grid(True, linewidth=0.3, alpha=0.4)
            ax.set_xlim(-0.06, 1.06)
            ax.set_xticks([0.0, 0.5, 1.0])
            ax.set_xticklabels(['0', '0.5', '1'])
            ax.yaxis.set_major_locator(plt.MaxNLocator(3))
            if col == 0:
                ax.set_ylabel(metric_label)
            if row == len(ROWS) - 1:
                ax.set_xlabel(risk_label)

    for row in range(len(ROWS)):
        core = np.array(core_values[row])
        core = core[~np.isnan(core)]
        margin = Y_MARGIN * np.ptp(core)
        low, high = core.min() - margin, core.max() + margin
        axes[row, 0].set_ylim(low - 0.02 * (high - low), high)

    # One-row legend above the panels: ARENA, the reference front, then the optimal solutions
    handles = [Patch(facecolor=(*plt.matplotlib.colors.to_rgb(ARENA_COLOR), ARENA_BAND_ALPHA), edgecolor=ARENA_COLOR,
                     linewidth=0.8, label='ARENA'),
               Line2D([], [], linestyle='none', marker='o', markersize=2.5, color=REFERENCE_STYLE['color'],
                      label='Ref. front')]
    handles += [Line2D([], [], color=c, linestyle=s, linewidth=0.8, label=l) for l, c, s, _ in optimal]
    legend = fig.legend(handles=handles, loc='upper center', ncol=len(handles), frameon=False, handlelength=1.5,
                        columnspacing=0.8, handletextpad=0.3, borderaxespad=0.0, bbox_to_anchor=(0.5, 1.0))
    print('ARENA: mean +- std over {} fronts'.format(len(plannings)))

    # Panels right below the legend
    fig.canvas.draw()
    legend_bottom = legend.get_window_extent().transformed(fig.transFigure.inverted()).y0
    fig.tight_layout(pad=0.15, h_pad=0.25, w_pad=0.25, rect=(0, 0, 1, legend_bottom - 0.005))
    fig.align_ylabels(axes[:, 0])  # The y axis titles aligned whatever the width of the tick labels
    # Legend centered on the panels (not on the figure, whose left part holds the y labels)
    panels_center = (axes[0, 0].get_position().x0 + axes[0, -1].get_position().x1) / 2.0
    legend.set_bbox_to_anchor((panels_center, 1.0), transform=fig.transFigure)
    for extension in ['pdf', 'png']:
        fig.savefig('{}.{}'.format(output, extension), dpi=args.dpi, bbox_inches='tight', pad_inches=0.02)
    print('Figure written to {0}.pdf and {0}.png'.format(output))


if __name__ == '__main__':
    main()
