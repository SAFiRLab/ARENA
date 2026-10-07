"""Computation-time and Pareto-quality analysis of the hyperparameters variation tests.

Two-panel figure (single column):
    (a) planning time vs every swept hyperparameter, relative to its default value (one curve per hyperparameter)
    (b) Pareto front quality vs the same x axis as (a) (--quality-x time: vs planning time), one curve per sweep:
        IGD+ on the left axis (solid lines) and hypervolume ratio on the right axis (dashed lines)

The quality metrics are computed with compute_pareto_metrics.py against the reference front, the non-dominated union
of the reference reports (reference_front.sh) and of the analysed runs. The HV reference point is the worst value of
all the fronts (--hv-ref-point worst) so that every feasible run has a non-null hypervolume.

The sample size sweep is not in panel (b) by default: the costs are evaluated with a different number of samples,
so its fronts are not directly comparable to a reference computed with another sample size. --sample-size-metrics-plot
adds it: its runs are scored against the same reference front, normalization and HV reference point as the other
sweeps, without being used to build them (the other curves are the same with or without it).

Benchmark planners (--baselines <label>=<report folder>, see compare_baselines.py): their mean planning time (+- std)
and their mean IGD+ and hypervolume ratio are drawn as horizontal lines (grey shades, one marker per planner), they don't
depend on the hyperparameters of ARENA. Their safe solutions are part of the reference front (non-dominated union), like
in compare_baselines.py, --baselines-scored-only scores them without adding them to the reference front.

Usage:
    python3 plot_computation_analysis.py [--sweeps <report folders>] [--reference <report folders>]
                                         [--baselines <label>=<report folder> ...] [--baselines-scored-only]
                                         [--onboard <report folders> --onboard-label <device>]
                                         [--quality-sweeps nb_of_generations population_size rrt_range]
                                         [--sample-size-metrics-plot] [--quality-x hyperparameter|time]
                                         [--output <file prefix>] [--fit] [--size 3.5 2.6] [--smooth 1] [--dpi 400]

Two separate figures are written: <prefix>_planning_time.pdf/png and <prefix>_pareto_quality.pdf/png
"""

import argparse
import os
import sys

import numpy as np
from matplotlib import pyplot as plt

from compute_pareto_metrics import evaluate, find_reports, load_runs
from plot_hyperparameter_variations import open_csv


HERE = os.path.dirname(os.path.realpath(__file__))

# Swept hyperparameter -> (label, default value of the planner)
HYPERPARAMETERS = {
    'nb_of_generations': (r'$N_{gen}$', 1000.0),
    'population_size': (r'$N_{pop}$', 40.0),
    'nurbs_sample_size': (r'$N_{nurbs}$', 50.0),
    'rrt_range': (r'$\delta_{RRT}$', 5.0),  # The order of this dictionary is the order of the curves and legends
}
COLORS = {'nb_of_generations': 'tab:blue', 'population_size': 'tab:green',
          'nurbs_sample_size': 'tab:red', 'rrt_range': 'tab:purple'}

DEFAULT_SWEEPS = [os.path.join(HERE, 'CL_map_3', r) for r in ['report1', 'report2', 'report3', 'report4']]
# Over-designed runs of reference_front.sh: report_3d_reference*.csv and their *_pareto_front.csv
DEFAULT_REFERENCE = [os.path.join(HERE, 'CL_map_3', 'reference_front')]
DEFAULT_QUALITY_SWEEPS = ['nb_of_generations', 'population_size', 'rrt_range']

# Figure style for a two-column IEEE paper: Times-like font, text >= 7 pt at the printed size, thick lines
PAPER_STYLE = {
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'Nimbus Roman', 'STIXGeneral', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 9,
    'xtick.labelsize': 7.5,
    'ytick.labelsize': 7.5,
    'legend.fontsize': 7.5,
    'axes.linewidth': 0.7,
    'xtick.major.width': 0.7,
    'ytick.major.width': 0.7,
    'lines.linewidth': 1.5,
    'pdf.fonttype': 42,  # Embedded TrueType fonts, required by IEEE PDF eXpress
    'ps.fonttype': 42,
}
LINE_WIDTH = 1.5
# Panel (b): the line style gives the metric (the curves have no markers, every setting is a point of the line)
IGD_STYLE = {'linestyle': '-'}
HV_STYLE = {'linestyle': (0, (3, 1.5))}
STD_ALPHA = 0.2  # Opacity of the standard deviation bands of panel (b) (they overlap)
# The planning time varies by ~3 % between the runs of a setting: more opaque band, drawn over the line of panel (a)
TIME_STD_ALPHA = 0.45
TIME_LINE_WIDTH = 0.9
MARKER_SIZE = 3.5
NB_OF_MARKERS = 10  # Markers per curve, the sweeps have up to ~100 settings

# Figure sizes in inches: one column of a two-column paper (panels stacked) or the full page width (side by side)
# Size of each figure in inches, the default is one column of a two-column paper
FIGURE_SIZE = (3.5, 2.6)

# Benchmark planners: (color, marker), in the order of --baselines
BASELINE_STYLES = [('black', 'o'), ('dimgray', 's'), ('gray', '^'), ('darkgray', 'D'), ('silver', 'v'), ('black', 'x')]
BASELINE_LINE_WIDTH = 0.9
BASELINE_STD_ALPHA = 0.12
NB_OF_BASELINE_MARKERS = 5


def load_baselines(values):
    """Returns [(label, runs)] of the --baselines arguments <label>=<report folder>."""
    baselines = []
    for value in values:
        if '=' not in value:
            sys.exit('--baselines expects <label>=<report folder>, got {}'.format(value))
        label, folder = value.split('=', 1)
        runs = []
        for report, pareto in find_reports([folder]):
            runs += load_runs(report, pareto)
        if not runs:
            print('Warning: no report for {} in {}, it is skipped'.format(label, folder), file=sys.stderr)
            continue
        baselines.append((label, runs))
    return baselines


def plot_baseline_lines(ax, baselines, value, linestyle, with_std=True, ylim_top=None):
    """Horizontal line of the mean of value(run) of every benchmark planner, with markers to tell them apart."""
    x_min, x_max = ax.get_xlim()
    log_x = ax.get_xscale() == 'log'
    x = np.geomspace(x_min, x_max, 50) if log_x else np.linspace(x_min, x_max, 50)
    for i, (label, runs) in enumerate(baselines):
        color, marker = BASELINE_STYLES[i % len(BASELINE_STYLES)]
        values = np.array([value(r) for r in runs], dtype=float)
        values = values[~np.isnan(values)]
        if len(values) == 0:
            continue
        mean, std = np.mean(values), (np.std(values, ddof=1) if len(values) > 1 else 0.0)
        if ylim_top is not None and mean > ylim_top:
            print('{}: {:.3f} is above the axis ({:.3f}), its line is not visible'.format(label, mean, ylim_top))
        # Markers shifted for every planner so that overlapping lines stay readable
        ax.plot(x, np.full_like(x, mean), color=color, linewidth=BASELINE_LINE_WIDTH, linestyle=linestyle, marker=marker,
                markersize=MARKER_SIZE, markerfacecolor='white', markeredgewidth=0.8,
                markevery=(i * 2 % 10, 50 // NB_OF_BASELINE_MARKERS), zorder=4)
        if with_std and std > 0.0:
            ax.fill_between(x, max(mean - std, 0.0), mean + std, color=color, alpha=BASELINE_STD_ALPHA, linewidth=0, zorder=2)
    ax.set_xlim(x_min, x_max)


def add_baselines_legend(ax, baselines, loc):
    """Legend of the benchmark planners (marker and grey shade), kept with the legend of the sweeps."""
    if not baselines:
        return
    first_legend = ax.get_legend()
    handles = []
    for i, (label, _) in enumerate(baselines):
        color, marker = BASELINE_STYLES[i % len(BASELINE_STYLES)]
        handles.append(ax.plot([], [], color=color, linewidth=BASELINE_LINE_WIDTH, marker=marker, markersize=MARKER_SIZE,
                               markerfacecolor='white', markeredgewidth=0.8, label=label)[0])
    ax.legend(handles=handles, loc=loc, frameon=False, handlelength=1.8, borderaxespad=0.2, labelspacing=0.3)
    if first_legend is not None:
        ax.add_artist(first_legend)


OBJECTIVE_COLUMNS = {'time': ('Time coefficient', 'Chosen time cost'),
                     'safety': ('Security coefficient', 'Chosen security cost'),
                     'energy': ('Energy coefficient', 'Chosen energy cost')}


def load_sweeps(folders):
    """Returns {swept hyperparameter: runs} for the report folders, the swept hyperparameter is detected."""
    sweeps = {}
    for report, pareto in find_reports(folders):
        runs = load_runs(report, pareto)
        varied = [h for h in HYPERPARAMETERS if len(set(r[h] for r in runs)) > 1]
        if len(varied) != 1:
            print('Warning: {} varies {}, it is skipped'.format(report, varied or 'nothing'), file=sys.stderr)
            continue
        sweeps.setdefault(varied[0], []).extend(runs)
    return sweeps


def per_setting(runs, hyperparameter, value):
    """Mean and standard deviation of value(run) for every setting of the swept hyperparameter."""
    settings = np.array(sorted(set(r[hyperparameter] for r in runs)), dtype=float)
    means, stds = [], []
    for setting in settings:
        values = np.array([value(r) for r in runs if r[hyperparameter] == setting], dtype=float)
        values = values[~np.isnan(values)]
        means.append(np.mean(values) if len(values) else np.nan)
        # Sample standard deviation of the runs of the setting
        stds.append(np.std(values, ddof=1) if len(values) > 1 else (0.0 if len(values) else np.nan))
    return settings, np.array(means), np.array(stds)


def smooth(values, window):
    """Moving average over window consecutive settings (shrinking window at the edges)."""
    if window <= 1:
        return values
    kernel = np.ones(window)
    valid = ~np.isnan(values)
    sums = np.convolve(np.where(valid, values, 0.0), kernel, mode='same')
    counts = np.convolve(valid.astype(float), kernel, mode='same')
    return np.where(counts > 0, sums / np.maximum(counts, 1.0), np.nan)


def markevery(nb_of_points):
    """Markers evenly spaced along the drawn curve (fraction of the axes diagonal), not by data index."""
    return 1 if nb_of_points <= NB_OF_MARKERS else 1.0 / NB_OF_MARKERS


def default_index(settings, hyperparameter):
    """Index of the setting closest to the default value."""
    return int(np.argmin(np.abs(np.log(settings / HYPERPARAMETERS[hyperparameter][1]))))


def plot_planning_time(ax, sweeps, onboard, onboard_label, baselines=()):
    for hyperparameter in [h for h in HYPERPARAMETERS if h in sweeps]:
        runs = sweeps[hyperparameter]
        label, default = HYPERPARAMETERS[hyperparameter]
        x, mean, std = per_setting(runs, hyperparameter, lambda r: r['planning_time'])
        color = COLORS[hyperparameter]
        # Thinner line than panel (b): the standard deviation band is about as thick as a 1.5 pt line
        ax.plot(x / default, mean, color=color, linewidth=TIME_LINE_WIDTH, label=label)
        # Mean +- the standard deviation of the runs of every setting
        ax.fill_between(x / default, np.maximum(mean - std, 0.0), mean + std, color=color, alpha=TIME_STD_ALPHA,
                        linewidth=0, zorder=3)

    for hyperparameter, runs in onboard.items():
        _, default = HYPERPARAMETERS[hyperparameter]
        x, mean, std = per_setting(runs, hyperparameter, lambda r: r['planning_time'])
        ax.errorbar(x / default, mean, yerr=std, color=COLORS[hyperparameter], marker='o', markersize=MARKER_SIZE + 1,
                    markerfacecolor='none', markeredgewidth=1.0, linestyle=':', linewidth=LINE_WIDTH, capsize=2)
    if onboard:
        ax.plot([], [], color='gray', marker='o', markerfacecolor='none', linestyle=':', label=onboard_label)

    ax.axvline(1.0, color='black', linewidth=0.8, linestyle='--')
    # The swept values span 0.001x to 5x their default, only the x axis is logarithmic
    ax.set_xscale('log')
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel('Hyperparameter value / default value')
    ax.set_ylabel('Planning time (s)')
    ax.legend(loc='upper left', frameon=False, handlelength=1.8, borderaxespad=0.2, labelspacing=0.3)
    ax.grid(True, which='major', linewidth=0.4, alpha=0.4)

    if baselines:
        plot_baseline_lines(ax, baselines, lambda r: r['planning_time'], '-', ylim_top=ax.get_ylim()[1])
        add_baselines_legend(ax, baselines, 'upper center')


def plot_quality(ax, sweeps, quality_sweeps, smoothing, quality_x, baselines=(), xlim=None):
    """Pareto quality of the sweeps.

    quality_x:
        hyperparameter: x = hyperparameter value / default value, same x axis as panel (a)
        time: x = mean planning time of the setting
    The default configuration is shown by a vertical dashed line, like in panel (a).
    """
    ax_hv = ax.twinx()
    default_times = []
    quality_sweeps = [h for h in HYPERPARAMETERS if h in quality_sweeps]
    for hyperparameter in quality_sweeps:
        if hyperparameter not in sweeps:
            continue
        runs = [r for r in sweeps[hyperparameter] if r['feasible']]
        color = COLORS[hyperparameter]
        settings, time_mean, _ = per_setting(runs, hyperparameter, lambda r: r['planning_time'])
        _, igd_mean, igd_std = per_setting(runs, hyperparameter, lambda r: r['igd_plus'])
        _, hv_mean, hv_std = per_setting(runs, hyperparameter, lambda r: r['hypervolume_ratio'])
        default_times.append(time_mean[default_index(settings, hyperparameter)])

        # Moving average over consecutive settings (sorted by setting value) to make the trends readable
        igd_s, hv_s = smooth(igd_mean, smoothing), smooth(hv_mean, smoothing)
        igd_std_s, hv_std_s = smooth(igd_std, smoothing), smooth(hv_std, smoothing)
        if quality_x == 'time':
            x = smooth(time_mean, smoothing)
            order = np.argsort(x)
        else:
            x = settings / HYPERPARAMETERS[hyperparameter][1]
            order = np.arange(len(x))

        # IGD+: solid line and filled circles, HV ratio: dashed line and hollow squares
        # Mean of the runs of every setting surrounded by their standard deviation
        ax.plot(x[order], igd_s[order], color=color, linewidth=LINE_WIDTH, **IGD_STYLE)
        ax.fill_between(x[order], np.maximum(igd_s - igd_std_s, 0.0)[order], (igd_s + igd_std_s)[order],
                        color=color, alpha=STD_ALPHA, linewidth=0)
        ax_hv.plot(x[order], hv_s[order], color=color, linewidth=LINE_WIDTH, **HV_STYLE)
        ax_hv.fill_between(x[order], np.maximum(hv_s - hv_std_s, 0.0)[order], np.minimum(hv_s + hv_std_s, 1.0)[order],
                           color=color, alpha=STD_ALPHA, linewidth=0)

    # The axis labels give the metric of each line style, the legend gives the swept hyperparameter of each color
    for hyperparameter in quality_sweeps:
        if hyperparameter in sweeps:
            ax.plot([], [], color=COLORS[hyperparameter], linewidth=LINE_WIDTH * 2,
                    label=HYPERPARAMETERS[hyperparameter][0])

    if quality_x == 'time':
        # Planning time of the default configuration (mean over the sweeps, they all contain it)
        ax.axvline(np.mean(default_times), color='black', linewidth=0.8, linestyle='--')
        ax.set_xlim(left=0.0)
        ax.set_xlabel('Planning time (s)')
    else:
        ax.axvline(1.0, color='black', linewidth=0.8, linestyle='--')
        ax.set_xscale('log')
        ax.set_xlabel('Hyperparameter value / default value')
    ax.set_ylim(bottom=0.0)
    ax.set_ylabel('IGD$^+$ (solid lines)')
    ax_hv.set_ylabel('Hypervolume ratio (dashed lines)')
    ax_hv.set_ylim(0.0, 1.05)
    ax.grid(True, which='major', linewidth=0.4, alpha=0.4)
    # Inside the axes, on the left between the IGD+ and HV curves of the generations sweep (free area of the plot)
    ax.legend(loc='center left', bbox_to_anchor=(0.03, 0.5), frameon=False,
              handlelength=1.5, handletextpad=0.4, borderaxespad=0.1, labelspacing=0.3)

    if xlim is not None:
        ax.set_xlim(xlim)
    if baselines:
        # Only the feasible runs, like the sweeps: IGD+ (solid lines) on the left axis, HV ratio (dashed) on the right one
        feasible = [(label, [r for r in runs if r['feasible']]) for label, runs in baselines]
        plot_baseline_lines(ax, feasible, lambda r: r['igd_plus'], IGD_STYLE['linestyle'], with_std=False)
        ax_hv.set_xlim(ax.get_xlim())
        plot_baseline_lines(ax_hv, feasible, lambda r: r['hypervolume_ratio'], HV_STYLE['linestyle'], with_std=False)
        add_baselines_legend(ax, baselines, 'upper right')


def check_reference(reference_folders, expected_runs):
    """True if the reference reports contain at least expected_runs runs in total.

    The cost weights only select the chosen solution of the final front, they don't change the NSGA-II optimization,
    so the Pareto fronts of the runs of every objective sample the same front and are counted together.
    """
    runs_per_objective = {objective: 0 for objective in OBJECTIVE_COLUMNS}
    for report, _ in find_reports(reference_folders):
        header, data = open_csv(report)
        ids = set(row[header.index('Id')] for row in data)
        for objective, (coefficient, _) in OBJECTIVE_COLUMNS.items():
            if data and float(data[0][header.index(coefficient)]) == 1.0:
                runs_per_objective[objective] += len(ids)
    total = sum(runs_per_objective.values())
    print('Reference runs: {} in total, per cost weights {}'.format(total, runs_per_objective))
    return total >= expected_runs


def print_reference_benchmarks(reference_folders):
    """Best observed (single-objective benchmark) costs of the reference reports."""
    for report, _ in find_reports(reference_folders):
        header, data = open_csv(report)
        rows, seen = [], set()
        for row in data:
            if row[header.index('Id')] in seen:
                continue
            seen.add(row[header.index('Id')])
            rows.append(row)

        for objective, (coefficient, cost) in OBJECTIVE_COLUMNS.items():
            if float(rows[0][header.index(coefficient)]) != 1.0:
                continue
            costs = np.array([float(r[header.index(cost)]) for r in rows if float(r[header.index('Feasible')]) == 1])
            print('Reference {} ({} runs, {} feasible): best observed {} cost {:.6g}, mean {:.6g} +- {:.6g}'.format(
                os.path.basename(report), len(rows), len(costs), objective, costs.min(), costs.mean(), costs.std()))


def print_runtime_model(sweeps):
    """Least-squares fit of t = c0 + c1 G N S + c2 G N + c3 G N^2 on the NSGA-II sweeps (optional sentence of the text)."""
    runs = [r for h in ['nb_of_generations', 'population_size', 'nurbs_sample_size'] for r in sweeps.get(h, [])]
    if not runs:
        return
    G, N, S, t = (np.array([r[k] for r in runs], dtype=float)
                  for k in ['nb_of_generations', 'population_size', 'nurbs_sample_size', 'planning_time'])
    X = np.stack([np.ones_like(G), G * N * S, G * N, G * N ** 2], axis=1)
    c, *_ = np.linalg.lstsq(X, t, rcond=None)
    prediction = X @ c
    r2 = 1.0 - np.sum((t - prediction) ** 2) / np.sum((t - t.mean()) ** 2)
    print('Runtime model: t = {:.3g} + {:.3g} G N S + {:.3g} G N + {:.3g} G N^2 s (R2 {:.3f}, median error {:.1f} %)'.format(
        *c, r2, 100.0 * np.median(np.abs(t - prediction) / t)))
    g, n, s = (HYPERPARAMETERS[h][1] for h in ['nb_of_generations', 'population_size', 'nurbs_sample_size'])
    terms = [c[0], c[1] * g * n * s, c[2] * g * n, c[3] * g * n ** 2]
    print('Default configuration: constant {:.3f} s, evaluation {:.3f} s, per individual {:.3f} s, sorting {:.3f} s, '
          'total {:.3f} s'.format(*terms, sum(terms)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sweeps', nargs='+', default=DEFAULT_SWEEPS, help='Report folders of the hyperparameter sweeps')
    parser.add_argument('--reference', nargs='*', default=DEFAULT_REFERENCE,
                        help='Report folders of the reference runs (reference_front.sh), none to draw panel (a) only')
    parser.add_argument('--onboard', nargs='*', default=[], help='Report folders of the sweeps run on the onboard hardware')
    parser.add_argument('--baselines', nargs='*', default=[],
                        help='<label>=<report folder> of the benchmark planners (compare_baselines.py), drawn as lines')
    parser.add_argument('--baselines-scored-only', action='store_true',
                        help='Score the benchmark planners without adding their solutions to the reference front')
    parser.add_argument('--onboard-label', default='Onboard', help='Legend of the onboard results')
    parser.add_argument('--quality-x', choices=['hyperparameter', 'time'], default='hyperparameter',
                        help='x axis of panel (b): hyperparameter value / default value (same as panel (a)) or planning time')
    parser.add_argument('--sample-size-metrics-plot', '--sample_size_metrics_plot', action='store_true',
                        help='Add the sample size sweep to panel (b), its costs are evaluated with its own sample sizes')
    parser.add_argument('--quality-sweeps', nargs='+', default=DEFAULT_QUALITY_SWEEPS, choices=list(HYPERPARAMETERS),
                        help='Sweeps shown in panel (b)')
    parser.add_argument('--output', default=os.path.join(HERE, 'computation_analysis'), help='Prefix of the figure files')
    parser.add_argument('--fit', action='store_true', help='Print the fitted runtime model')
    parser.add_argument('--size', nargs=2, type=float, default=list(FIGURE_SIZE), metavar=('WIDTH', 'HEIGHT'),
                        help='Size of each figure in inches (default: one column, 3.5 x 2.6)')
    parser.add_argument('--smooth', type=int, default=1,
                        help='Panel (b): moving average over this number of consecutive settings (1: none)')
    parser.add_argument('--dpi', type=int, default=400, help='Resolution of the PNG')
    parser.add_argument('--reference-runs', type=int, default=1,
                        help='Number of reference runs (all cost weights together) expected before panel (b) is drawn')
    parser.add_argument('--allow-partial-reference', action='store_true',
                        help='Draw panel (b) even if the reference runs are not complete (preview only)')
    args = parser.parse_args()

    sweeps = load_sweeps(args.sweeps)
    if not sweeps:
        sys.exit('No sweep report found')
    onboard = load_sweeps(args.onboard) if args.onboard else {}
    baselines = load_baselines(args.baselines)
    baseline_runs = [r for _, runs in baselines for r in runs]

    # Panel (b) needs the complete reference front (reference_front.sh)
    reference_runs = []
    for report, pareto in find_reports(args.reference):
        reference_runs += load_runs(report, pareto, all_safe_solutions=True)
    draw_quality = bool(reference_runs)
    if not reference_runs:
        print('No reference report (--reference): only panel (a) is drawn')
    elif not check_reference(args.reference, args.reference_runs):
        if args.allow_partial_reference:
            print('Warning: the reference runs are not complete, panel (b) is a preview only', file=sys.stderr)
        else:
            print('The reference runs are not complete: only panel (a) is drawn (--allow-partial-reference to preview)')
            draw_quality = False

    if draw_quality:
        # Quality metrics with a common normalization for all the sweeps of panel (b)
        quality_runs = [r for h in args.quality_sweeps for r in sweeps.get(h, [])]
        # The sample size sweep is scored without being part of the reference front and of the normalization
        scored_runs = []
        if args.sample_size_metrics_plot and 'nurbs_sample_size' not in args.quality_sweeps:
            scored_runs = sweeps.get('nurbs_sample_size', [])
            args.quality_sweeps = list(args.quality_sweeps) + ['nurbs_sample_size']
            print('Sample size sweep added to panel (b): {} runs scored against the same reference'.format(len(scored_runs)))
        # The benchmark planners are part of the reference front (non-dominated union) unless --baselines-scored-only
        if args.baselines_scored_only:
            scored_runs = list(scored_runs) + baseline_runs
        else:
            quality_runs = quality_runs + baseline_runs
        _, info = evaluate(quality_runs, reference_runs, 1.1, 'worst', scored_runs)
        print('Reference front: {} points'.format(info['reference_size']))

    for label, runs in baselines:
        times = np.array([r['planning_time'] for r in runs])
        line = '{}: {} runs ({} feasible), planning time {:.3f} +- {:.3f} s'.format(
            label, len(runs), sum(r['feasible'] for r in runs), times.mean(), times.std(ddof=1) if len(times) > 1 else 0.0)
        if draw_quality:
            feasible = [r for r in runs if r['feasible']]
            line += ', IGD+ {:.3f}, HV ratio {:.3f}'.format(np.nanmean([r['igd_plus'] for r in feasible]),
                                                           np.nanmean([r['hypervolume_ratio'] for r in feasible]))
        print(line)
    for hyperparameter, runs in sweeps.items():
        x, mean, std = per_setting(runs, hyperparameter, lambda r: r['planning_time'])
        print('{}: {} runs, planning time {:.2f} s ({:g}) to {:.2f} s ({:g})'.format(
            hyperparameter, len(runs), mean[0], x[0], mean[-1], x[-1]))
    if reference_runs:
        print_reference_benchmarks(args.reference)
    if args.fit:
        print_runtime_model(sweeps)

    plt.rcParams.update(PAPER_STYLE)
    figures = {}

    fig_time, ax_time = plt.subplots(figsize=args.size)
    plot_planning_time(ax_time, sweeps, onboard, args.onboard_label, baselines)
    figures['planning_time'] = fig_time

    if draw_quality:
        fig_quality, ax_quality = plt.subplots(figsize=args.size)
        # Same x axis as the planning time figure, a setting is at the same position in both figures
        plot_quality(ax_quality, sweeps, args.quality_sweeps, args.smooth, args.quality_x, baselines,
                     ax_time.get_xlim() if args.quality_x == 'hyperparameter' else None)
        if args.quality_x == 'hyperparameter':
            # Same x axis as the planning time figure, a setting is at the same position in both figures
            ax_quality.set_xlim(ax_time.get_xlim())
        figures['pareto_quality'] = fig_quality

    # Every figure is saved at its printed size (include it with width=\\columnwidth), the legend of the Pareto
    # quality figure is above its axes so the bounding box is tightened around the content
    for name, fig in figures.items():
        fig.tight_layout(pad=0.3)
        for extension in ['pdf', 'png']:
            fig.savefig('{}_{}.{}'.format(args.output, name, extension), dpi=args.dpi, bbox_inches='tight', pad_inches=0.02)
        print('Figure written to {0}_{1}.pdf and {0}_{1}.png'.format(args.output, name))
    plt.show()

if __name__ == '__main__':
    main()
