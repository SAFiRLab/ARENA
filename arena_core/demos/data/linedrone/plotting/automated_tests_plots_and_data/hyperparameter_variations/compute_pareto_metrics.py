"""Pareto front quality metrics over repeated testbench runs.

Reads the reports written by testbench_node (report_3d_<world>_<date>.csv and its
report_3d_<world>_<date>_pareto_front.csv) and computes, for every planning (run):
    - Hypervolume (HV) and HV ratio to the reference front
    - Additive epsilon indicator (I_eps+) to the reference front
    - Inverted generational distance (IGD) and IGD+ to the reference front
    - Spacing (Schott)
    - Number of non-dominated solutions
The runs are then grouped by setting (hyperparameters or cost coefficients) and the
mean, standard deviation, median and interquartile range are reported for every group.

All the objectives are minimized (time, security and energy costs).

Reference front and normalization:
    The true Pareto front is unknown, the reference front is the non-dominated set of the union
    of all the fronts analysed (optionally completed with the fronts of other reports given with
    --reference, e.g. long runs with many generations). The objectives are normalized with the
    ideal and nadir points of the reference front, the HV reference point is (1.1, 1.1, 1.1)
    in the normalized space. The same normalization is used for every run so their values are comparable.

Usage:
    python3 compute_pareto_metrics.py <report.csv or folder> [...] [--group-by hyperparameters|coefficients]
                                      [--reference <report.csv or folder> ...] [--ref-point 1.1] [--hv-ref-point fixed|worst]
                                      [--output <prefix>] [--plot <metric>]
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np


OBJECTIVES = ['Time cost', 'Security cost', 'Energy cost']
METRICS = ['hypervolume', 'hypervolume_ratio', 'epsilon_additive', 'igd', 'igd_plus', 'spacing', 'front_size']
# Computation values of every run, in seconds (NaN when the report doesn't have them)
TIMINGS = ['planning_time', 'initialization_time', 'optimization_time', 'nb_of_control_points']
SUMMARY_VALUES = METRICS + TIMINGS


# ----------------------------------------------------------------------------------------------- #
# Pareto utilities
# ----------------------------------------------------------------------------------------------- #
def non_dominated(points):
    """Returns the non-dominated points (minimization), duplicates are kept once."""
    points = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(points) == 0:
        return points
    keep = np.ones(len(points), dtype=bool)
    for i, p in enumerate(points):
        if not keep[i]:
            continue
        # Points dominated by p
        dominated = np.all(p <= points, axis=1) & np.any(p < points, axis=1)
        keep &= ~dominated
    return points[keep]


def hypervolume_2d(points, ref):
    """Exact hypervolume of 2D points (minimization) bounded by ref."""
    points = points[np.all(points < ref, axis=1)]
    if len(points) == 0:
        return 0.0
    points = points[np.argsort(points[:, 0])]
    hv = 0.0
    best_y = ref[1]
    for x, y in points:
        if y < best_y:
            hv += (ref[0] - x) * (best_y - y)
            best_y = y
    return hv


def hypervolume_3d(points, ref):
    """Exact hypervolume of 3D points (minimization) bounded by ref, computed by slicing along the 3rd objective."""
    points = np.asarray(points, dtype=float)
    points = points[np.all(points < ref, axis=1)]
    if len(points) == 0:
        return 0.0
    points = points[np.argsort(points[:, 2])]
    hv = 0.0
    for i in range(len(points)):
        z_next = points[i + 1, 2] if i + 1 < len(points) else ref[2]
        depth = z_next - points[i, 2]
        if depth > 0.0:
            hv += hypervolume_2d(points[:i + 1, :2], ref[:2]) * depth
    return hv


def epsilon_additive(front, reference):
    """Smallest eps such that every reference point is weakly dominated by a point of front shifted by eps."""
    diff = front[:, None, :] - reference[None, :, :]       # (n_front, n_ref, n_obj)
    return float(np.max(np.min(np.max(diff, axis=2), axis=0)))


def igd(front, reference):
    """Mean euclidean distance from every reference point to its closest point of front."""
    dist = np.linalg.norm(front[:, None, :] - reference[None, :, :], axis=2)
    return float(np.mean(np.min(dist, axis=0)))


def igd_plus(front, reference):
    """IGD+ (Ishibuchi et al. 2015), weakly Pareto compliant version of IGD."""
    diff = np.maximum(front[:, None, :] - reference[None, :, :], 0.0)
    dist = np.sqrt(np.sum(diff ** 2, axis=2))
    return float(np.mean(np.min(dist, axis=0)))


def spacing(front):
    """Schott's spacing: standard deviation of the L1 distance of every point to its closest neighbour."""
    if len(front) < 2:
        return float('nan')
    dist = np.sum(np.abs(front[:, None, :] - front[None, :, :]), axis=2)
    np.fill_diagonal(dist, np.inf)
    d = np.min(dist, axis=1)
    return float(np.sqrt(np.sum((d - d.mean()) ** 2) / (len(d) - 1)))


# ----------------------------------------------------------------------------------------------- #
# Reports loading
# ----------------------------------------------------------------------------------------------- #
def find_reports(paths):
    """Returns the (report, pareto_front) file pairs found in the given files or folders."""
    reports = []
    for path in paths:
        if os.path.isdir(path):
            # Only the testbench reports, not the outputs of this script
            candidates = sorted(glob.glob(os.path.join(path, 'report_3d*.csv')))
        else:
            candidates = [path]
        for candidate in candidates:
            if candidate.endswith('_pareto_front.csv'):
                # A Pareto front file given directly: use its report
                report = candidate[:-len('_pareto_front.csv')] + '.csv'
                if not os.path.isfile(report):
                    print('Warning: {} needs its report {} (written next to it by testbench_node), it is skipped'
                          .format(candidate, report), file=sys.stderr)
                    continue
                if os.path.isdir(path):
                    continue  # The report of the folder is handled with the report itself
                candidate = report
            pareto = candidate[:-4] + '_pareto_front.csv'
            if not os.path.isfile(pareto):
                print('Warning: no Pareto front file for {}, it is skipped'.format(candidate), file=sys.stderr)
                continue
            if (candidate, pareto) not in reports:
                reports.append((candidate, pareto))
    return reports


def read_csv(path):
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        return list(reader)


def optional_float(row, column, scale=1.0):
    """Value of a column added in later versions of the reports, NaN if the report doesn't have it."""
    value = row.get(column)
    return float(value) * scale if value not in (None, '') else float('nan')


def load_runs(report_path, pareto_path):
    """Returns a dict Id -> run with its settings, feasibility and Pareto front."""
    runs = {}
    for row in read_csv(report_path):
        run_id = int(row['Id'])
        if run_id in runs:
            continue
        if 'Feasible' not in row:
            raise ValueError('{} has no Feasible column, it has not been written by the hyperparameters/steps '
                             'variation tests of testbench_node'.format(report_path))
        runs[run_id] = {
            'file': os.path.basename(report_path),
            'id': run_id,
            'nb_of_generations': int(float(row['Number of generations'])),
            'population_size': int(float(row['Population size'])),
            'nurbs_sample_size': int(float(row['Nurbs sample size'])),
            # Reports written before the RRT range sweep don't have it, -1 means not recorded
            'rrt_range': float(row['RRT range']) if row.get('RRT range') not in (None, '') else -1.0,
            'time_coefficient': float(row['Time coefficient']),
            'security_coefficient': float(row['Security coefficient']),
            'energy_coefficient': float(row['Energy coefficient']),
            'feasible': int(float(row['Feasible'])) == 1,
            'planning_time': float(row['Planing Time']) * 1.0e-9,
            'initialization_time': optional_float(row, 'Initialization time', 1.0e-9),
            'optimization_time': optional_float(row, 'Optimization time', 1.0e-9),
            'nb_of_control_points': optional_float(row, 'Number of control points'),
            'front': [],
        }

    for row in read_csv(pareto_path):
        run_id = int(row['Id'])
        if run_id in runs:
            runs[run_id]['front'].append([float(row[o]) for o in OBJECTIVES])

    for run in runs.values():
        run['front'] = np.array(run['front'], dtype=float).reshape(-1, len(OBJECTIVES))
    return list(runs.values())


def group_key(run, group_by):
    if group_by == 'coefficients':
        return (run['time_coefficient'], run['security_coefficient'], run['energy_coefficient'])
    return (run['nb_of_generations'], run['population_size'], run['nurbs_sample_size'], run['rrt_range'])


# ----------------------------------------------------------------------------------------------- #
# Evaluation
# ----------------------------------------------------------------------------------------------- #
def evaluate(runs, reference_runs, ref_point_value, ref_point_mode='fixed', scored_runs=()):
    """Computes the metrics of every run.

    scored_runs: other runs whose metrics are computed against the same reference front, normalization and HV
                 reference point, without being used to build them (e.g. runs whose costs are evaluated differently)

    ref_point_mode:
        fixed: the HV reference point is ref_point_value on every normalized objective
        worst: the HV reference point is ref_point_value times the worst normalized value of all the fronts
               (analysed and reference), so every feasible run has a non-null hypervolume
    """
    fronts = [r['front'] for r in runs + reference_runs if r['feasible'] and len(r['front']) > 0]
    if not fronts:
        raise ValueError('No feasible Pareto front found')

    reference = non_dominated(np.vstack(fronts))
    ideal = reference.min(axis=0)
    nadir = reference.max(axis=0)
    scale = np.where(nadir - ideal > 0.0, nadir - ideal, 1.0)
    normalize = lambda f: (f - ideal) / scale

    reference_n = normalize(reference)
    if ref_point_mode == 'worst':
        worst = np.max(np.vstack([normalize(f) for f in fronts]), axis=0)
        ref_point = ref_point_value * np.maximum(worst, 1.0)
    else:
        ref_point = np.full(len(OBJECTIVES), ref_point_value)
    reference_hv = hypervolume_3d(reference_n, ref_point)

    for run in list(runs) + list(scored_runs):
        if not run['feasible'] or len(run['front']) == 0:
            run.update({'hypervolume': 0.0, 'hypervolume_ratio': 0.0, 'epsilon_additive': float('nan'),
                        'igd': float('nan'), 'igd_plus': float('nan'), 'spacing': float('nan'), 'front_size': 0})
            continue
        front_n = normalize(non_dominated(run['front']))
        hv = hypervolume_3d(front_n, ref_point)
        run.update({
            'hypervolume': hv,
            'hypervolume_ratio': hv / reference_hv if reference_hv > 0.0 else float('nan'),
            'epsilon_additive': epsilon_additive(front_n, reference_n),
            'igd': igd(front_n, reference_n),
            'igd_plus': igd_plus(front_n, reference_n),
            'spacing': spacing(front_n),
            'front_size': len(front_n),
        })

    info = {'reference_size': len(reference), 'ideal': ideal, 'nadir': nadir, 'reference_hypervolume': reference_hv,
            'ref_point': ref_point}
    return reference, info


def summarize(runs, group_by):
    groups = {}
    for run in runs:
        groups.setdefault(group_key(run, group_by), []).append(run)

    summary = []
    for key in sorted(groups):
        group = groups[key]
        entry = {'setting': key, 'nb_of_runs': len(group), 'nb_of_feasible_runs': sum(r['feasible'] for r in group)}
        for metric in SUMMARY_VALUES:
            # Infeasible runs count as a null hypervolume, the distance metrics are only defined for feasible runs
            values = np.array([r[metric] for r in group], dtype=float)
            values = values[~np.isnan(values)]
            if len(values) == 0:
                stats = [float('nan')] * 5
            else:
                stats = [np.mean(values), np.std(values, ddof=1) if len(values) > 1 else 0.0, np.median(values),
                         np.percentile(values, 25), np.percentile(values, 75)]
            for name, value in zip(['mean', 'std', 'median', 'q1', 'q3'], stats):
                entry['{}_{}'.format(metric, name)] = value
        summary.append(entry)
    return summary


def setting_columns(group_by):
    if group_by == 'coefficients':
        return ['Time coefficient', 'Security coefficient', 'Energy coefficient']
    return ['Number of generations', 'Population size', 'Nurbs sample size', 'RRT range']


def write_outputs(prefix, runs, summary, reference, group_by):
    with open(prefix + '_runs.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['File', 'Id'] + setting_columns(group_by) + ['Feasible'] + SUMMARY_VALUES)
        for run in sorted(runs, key=lambda r: (r['file'], r['id'])):
            writer.writerow([run['file'], run['id']] + list(group_key(run, group_by)) + [int(run['feasible'])] +
                            [run[m] for m in SUMMARY_VALUES])

    with open(prefix + '_summary.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        stat_columns = ['{}_{}'.format(m, s) for m in SUMMARY_VALUES for s in ['mean', 'std', 'median', 'q1', 'q3']]
        writer.writerow(setting_columns(group_by) + ['Number of runs', 'Number of feasible runs'] + stat_columns)
        for entry in summary:
            writer.writerow(list(entry['setting']) + [entry['nb_of_runs'], entry['nb_of_feasible_runs']] +
                            [entry[c] for c in stat_columns])

    with open(prefix + '_reference_front.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(OBJECTIVES)
        writer.writerows(reference.tolist())


def plot_metric(summary, metric, group_by):
    from matplotlib import pyplot as plt

    if group_by == 'coefficients':
        x = np.arange(len(summary))
        labels = ['{:.2f}/{:.2f}/{:.2f}'.format(*e['setting']) for e in summary]
        xlabel = 'Coefficients (time/security/energy)'
    else:
        labels = None
        settings = np.array([e['setting'] for e in summary], dtype=float)
        varying = [i for i in range(settings.shape[1]) if len(np.unique(settings[:, i])) > 1]
        if len(varying) == 1:
            # A single hyperparameter is varied, use it as x axis
            x = settings[:, varying[0]]
            xlabel = setting_columns(group_by)[varying[0]] + (' (m)' if varying[0] == 3 else '')
        else:
            # Same x axis as plot_hyperparameter_variations.py
            x = np.prod(settings[:, :3], axis=1)
            xlabel = 'Number of points'
        order = np.argsort(x)
        x = x[order]
        summary = [summary[i] for i in order]

    mean = np.array([e[metric + '_mean'] for e in summary])
    std = np.array([e[metric + '_std'] for e in summary])
    plt.plot(x, mean, color='blue', label=metric)
    plt.fill_between(x, mean - std, mean + std, color='blue', alpha=0.2)
    if labels:
        plt.xticks(x, labels, rotation=90)
    plt.xlabel(xlabel)
    plt.ylabel(metric)
    plt.title('Mean and standard deviation of {}'.format(metric))
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('reports', nargs='+', help='Report files or folders containing reports')
    parser.add_argument('--group-by', choices=['hyperparameters', 'coefficients'], default='hyperparameters')
    parser.add_argument('--reference', nargs='*', default=[], help='Other reports whose fronts complete the reference front')
    parser.add_argument('--ref-point', type=float, default=1.1, help='HV reference point in the normalized space '
                        '(with --hv-ref-point worst: factor applied to the worst normalized values)')
    parser.add_argument('--hv-ref-point', choices=['fixed', 'worst'], default='fixed',
                        help='fixed: (ref-point, ref-point, ref-point), worst: ref-point times the worst values of all '
                        'the fronts, so every feasible run has a non-null hypervolume')
    parser.add_argument('--output', help='Prefix of the output files (default: next to the first report)')
    parser.add_argument('--plot', choices=METRICS, help='Plot the mean and standard deviation of a metric')
    args = parser.parse_args()

    runs = []
    for report, pareto in find_reports(args.reports):
        runs += load_runs(report, pareto)
    if not runs:
        sys.exit('No report found')

    reference_runs = []
    for report, pareto in find_reports(args.reference):
        reference_runs += load_runs(report, pareto)

    reference, info = evaluate(runs, reference_runs, args.ref_point, args.hv_ref_point)
    summary = summarize(runs, args.group_by)

    prefix = args.output or os.path.join(os.path.dirname(os.path.abspath(find_reports(args.reports)[0][0])), 'pareto_metrics')
    write_outputs(prefix, runs, summary, reference, args.group_by)

    print('Runs: {} ({} feasible)'.format(len(runs), sum(r['feasible'] for r in runs)))
    print('Reference front: {} points, ideal {}, nadir {}, normalized HV {:.6f}, HV reference point {}'.format(
        info['reference_size'], info['ideal'], info['nadir'], info['reference_hypervolume'], np.round(info['ref_point'], 3)))
    print('Results written to {}_runs.csv, {}_summary.csv and {}_reference_front.csv'.format(prefix, prefix, prefix))
    for entry in summary:
        print('{}: runs {} feasible {} | HV ratio {:.4f} +- {:.4f} | eps+ {:.4f} +- {:.4f} | IGD+ {:.4f} +- {:.4f} | '
              'spacing {:.4f} +- {:.4f} | size {:.1f}'.format(
                  entry['setting'], entry['nb_of_runs'], entry['nb_of_feasible_runs'],
                  entry['hypervolume_ratio_mean'], entry['hypervolume_ratio_std'],
                  entry['epsilon_additive_mean'], entry['epsilon_additive_std'],
                  entry['igd_plus_mean'], entry['igd_plus_std'],
                  entry['spacing_mean'], entry['spacing_std'], entry['front_size_mean']))

    if args.plot:
        plot_metric(summary, args.plot, args.group_by)


if __name__ == '__main__':
    main()
