"""Comparison of ARENA with the benchmark planners on common output metrics (table of the paper).

Reads the reports written by testbench_node for every planner (scripts/linedrone/automated_tests/ksl_airport_1/baselines,
copied to automated_tests_plots_and_data/baselines/CL_map_3/<planner>/) and computes for every planner, over its runs:
    - success rate: share of the runs with a feasible and safe trajectory (constraints and safety check of ARENA)
    - planning time (s)
    - costs of the chosen trajectory: time (ARENA time objective, s), safety and energy (J) costs
    - clearance of the chosen trajectory: minimum and mean distance from its samples to the closest occupied voxel of the
      inflated octomap (testbench_node::getClosestObstacleDistance, the distance to the obstacles is about this
      distance + the inflation radius, see --inflation-offset)
    - best value of every objective in the front of the run (time, safety and energy of the safe solutions)
    - Pareto front quality against the reference front: hypervolume ratio and IGD+ (compute_pareto_metrics.py)
The single path planners (A*, RRT*) have a front of one solution.

Reference front: the non-dominated union of the reference runs of ARENA (reference_front.sh) and of the safe solutions of
every planner compared (the true front is unknown, every planner can contribute to it). --no-union scores the
benchmark planners against the reference front of ARENA only (they don't contribute to it nor to the normalization).

The metrics of Fig. 4 (path duration at 2 m/s, mean clearance, energy of the chosen trajectory, as written by the risks
report of testbench_node) are also written for every planner (<output>_fig4_metrics.csv), the planners that don't adapt
to the risks are constant lines of Fig. 4.

Usage:
    python3 compare_baselines.py [--methods <label>=<report folder> ...] [--reference <report folders>]
                                 [--no-union] [--inflation-offset 0.0] [--output <prefix>]
"""

import argparse
import csv
import os
import sys

import numpy as np

from compute_pareto_metrics import evaluate, find_reports, load_runs


HERE = os.path.dirname(os.path.realpath(__file__))
BASELINES = os.path.join(HERE, '..', 'baselines', 'CL_map_3')

DEFAULT_METHODS = [
    ('ARENA', os.path.join(BASELINES, 'arena_default')),
    ('A*', os.path.join(BASELINES, 'astar')),
    ('MOAR-3D', os.path.join(BASELINES, 'moar3d_sweep')),
    ('RRT* (0.75 s)', os.path.join(BASELINES, 'rrt_star_short')),
    ('RRT* (5 s)', os.path.join(BASELINES, 'rrt_star_long')),
    ('Spline NSGA-II', os.path.join(BASELINES, 'spline_nsga2')),
]
DEFAULT_REFERENCE = [os.path.join(HERE, 'CL_map_3', 'reference_front')]

# Speed used by the risks report of testbench_node for the path duration of Fig. 4
FIG4_SPEED = 2.0

# Column name -> (header of the table, value of a run, number of decimals)
COLUMNS = [
    ('success_rate', 'Success (%)', 0),
    ('planning_time', 'Planning time (s)', 3),
    ('chosen_time', 'Duration (s)', 1),
    ('chosen_safety', 'Safety cost', 3),
    ('chosen_energy', 'Energy (kJ)', 1),
    ('min_clearance', 'Min. clearance (m)', 2),
    ('mean_clearance', 'Mean clearance (m)', 2),
    ('best_time', 'Best duration (s)', 1),
    ('best_safety', 'Best safety cost', 3),
    ('best_energy', 'Best energy (kJ)', 1),
    ('front_size', 'Front size', 1),
    ('hypervolume_ratio', 'HV ratio', 3),
    ('igd_plus', 'IGD+', 3),
]


def path_metrics(report_path, inflation_offset):
    """Clearance and length of the chosen path of every run (Id) of a planning report."""
    paths = {}
    with open(report_path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            if int(float(row['Feasible'])) != 1:
                continue
            paths.setdefault(int(row['Id']), []).append(
                [float(row['Drone Position X']), float(row['Drone Position Y']), float(row['Drone Position Z']),
                 float(row['Closest Obstacle Distance']) + inflation_offset])

    metrics = {}
    for run_id, rows in paths.items():
        rows = np.array(rows)
        length = np.sum(np.linalg.norm(np.diff(rows[:, :3], axis=0), axis=1))
        metrics[run_id] = {'min_clearance': rows[:, 3].min(), 'mean_clearance': rows[:, 3].mean(), 'length': length}
    return metrics


def chosen_costs(report_path):
    """Costs of the chosen trajectory of every run (Id) of a planning report."""
    costs = {}
    with open(report_path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            run_id = int(row['Id'])
            if run_id not in costs:
                costs[run_id] = [float(row['Chosen time cost']), float(row['Chosen security cost']),
                                 float(row['Chosen energy cost'])]
    return costs


def load_method(folder, inflation_offset):
    """Runs of a planner with their front (compute_pareto_metrics.load_runs) and the values of its chosen path."""
    runs = []
    for report, pareto in find_reports([folder]):
        report_runs = load_runs(report, pareto)
        paths = path_metrics(report, inflation_offset)
        costs = chosen_costs(report)
        for run in report_runs:
            run.update({key: float('nan') for key in ['min_clearance', 'mean_clearance', 'length', 'chosen_time',
                                                      'chosen_safety', 'chosen_energy']})
            if run['feasible']:
                run.update(paths.get(run['id'], {}))
                run['chosen_time'], run['chosen_safety'], run['chosen_energy'] = costs[run['id']]
                run['chosen_energy'] *= 1.0e-3
        runs += report_runs
    return runs


def best_in_front(run, objective):
    if not run['feasible'] or len(run['front']) == 0:
        return float('nan')
    return float(np.min(run['front'][:, objective]))


def summarize(label, runs):
    entry = {'method': label, 'nb_of_runs': len(runs), 'nb_of_feasible_runs': sum(r['feasible'] for r in runs)}
    entry['success_rate'] = (100.0 * entry['nb_of_feasible_runs'] / len(runs), 0.0)
    for run in runs:
        run['best_time'] = best_in_front(run, 0)
        run['best_safety'] = best_in_front(run, 1)
        run['best_energy'] = best_in_front(run, 2) * 1.0e-3
    for key, _, _ in COLUMNS:
        if key == 'success_rate':
            continue
        # The planning time counts every run, the other values only the feasible runs (HV of an infeasible run is 0)
        values = np.array([r[key] for r in runs if r['feasible'] or key == 'planning_time'], dtype=float)
        values = values[~np.isnan(values)]
        entry[key] = (np.mean(values), np.std(values, ddof=1) if len(values) > 1 else 0.0) if len(values) else (np.nan, np.nan)

    feasible = [r for r in runs if r['feasible']]
    entry['fig4'] = {
        'path_duration': np.mean([r['length'] / FIG4_SPEED for r in feasible]) if feasible else np.nan,
        'mean_clearance': np.mean([r['mean_clearance'] for r in feasible]) if feasible else np.nan,
        'min_clearance': np.mean([r['min_clearance'] for r in feasible]) if feasible else np.nan,
        'energy': np.mean([r['chosen_energy'] * 1.0e3 for r in feasible]) if feasible else np.nan,
    }
    return entry


def format_value(value, decimals, latex=False):
    mean, std = value
    if np.isnan(mean):
        return '--' if latex else ''
    if std == 0.0 or np.isnan(std):
        return '{:.{d}f}'.format(mean, d=decimals)
    if latex:
        return '{:.{d}f} $\\pm$ {:.{d}f}'.format(mean, std, d=decimals)
    return '{:.{d}f} +- {:.{d}f}'.format(mean, std, d=decimals)


def write_outputs(prefix, entries):
    with open(prefix + '.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Method', 'Runs', 'Feasible runs'] +
                        [h + suffix for _, h, _ in COLUMNS for suffix in [' mean', ' std']])
        for e in entries:
            writer.writerow([e['method'], e['nb_of_runs'], e['nb_of_feasible_runs']] +
                            [v for key, _, _ in COLUMNS for v in e[key]])

    with open(prefix + '.tex', 'w') as f:
        f.write('% Generated by compare_baselines.py\n')
        f.write('\\begin{tabular}{l' + 'c' * len(COLUMNS) + '}\n\\toprule\n')
        f.write('Method & ' + ' & '.join(h for _, h, _ in COLUMNS) + ' \\\\\n\\midrule\n')
        for e in entries:
            f.write(e['method'].replace('*', '$^*$') + ' & ' +
                    ' & '.join(format_value(e[key], d, latex=True) for key, _, d in COLUMNS) + ' \\\\\n')
        f.write('\\bottomrule\n\\end{tabular}\n')

    with open(prefix + '_fig4_metrics.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Method', 'Path Duration', 'Average Closest Obstacle Distance', 'Min Closest Obstacle Distance',
                         'Energy Cost'])
        for e in entries:
            writer.writerow([e['method'], e['fig4']['path_duration'], e['fig4']['mean_clearance'],
                             e['fig4']['min_clearance'], e['fig4']['energy']])


def parse_methods(values):
    methods = []
    for value in values:
        if '=' not in value:
            sys.exit('--methods expects <label>=<report folder>, got {}'.format(value))
        label, folder = value.split('=', 1)
        methods.append((label, folder))
    return methods


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--methods', nargs='+', help='<label>=<report folder> of every planner (default: ARENA and the '
                        'benchmark planners in ../baselines/CL_map_3)')
    parser.add_argument('--reference', nargs='*', default=DEFAULT_REFERENCE, help='Report folders of the reference runs')
    parser.add_argument('--no-union', action='store_true', help='The planners compared are not part of the reference front')
    parser.add_argument('--inflation-offset', type=float, default=0.0,
                        help='Added to the clearances (distance to the inflated voxels), e.g. the inflation radius 1.0 m')
    parser.add_argument('--output', default=os.path.join(BASELINES, 'baselines_table'), help='Prefix of the output files')
    args = parser.parse_args()

    methods = parse_methods(args.methods) if args.methods else DEFAULT_METHODS
    method_runs = []
    for label, folder in methods:
        if not find_reports([folder]):
            print('Warning: no report for {} in {}, it is skipped'.format(label, folder), file=sys.stderr)
            continue
        method_runs.append((label, load_method(folder, args.inflation_offset)))
    if not method_runs:
        sys.exit('No report found')

    reference_runs = []
    for report, pareto in find_reports(args.reference):
        reference_runs += load_runs(report, pareto, all_safe_solutions=True)

    all_runs = [r for _, runs in method_runs for r in runs]
    if args.no_union:
        arena_runs = [r for label, runs in method_runs if label == 'ARENA' for r in runs]
        others = [r for label, runs in method_runs if label != 'ARENA' for r in runs]
        _, info = evaluate(arena_runs, reference_runs, 1.1, 'worst', others)
    else:
        _, info = evaluate(all_runs, reference_runs, 1.1, 'worst')
    print('Reference front: {} points, ideal {}, nadir {}'.format(info['reference_size'], np.round(info['ideal'], 3),
                                                                 np.round(info['nadir'], 3)))

    entries = [summarize(label, runs) for label, runs in method_runs]
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    write_outputs(args.output, entries)

    for e in entries:
        print('{:<16} runs {:>3} ({:>3} feasible) | '.format(e['method'], e['nb_of_runs'], e['nb_of_feasible_runs']) +
              ' | '.join('{} {}'.format(h, format_value(e[key], d)) for key, h, d in COLUMNS))
    print('Written: {0}.csv, {0}.tex and {0}_fig4_metrics.csv'.format(args.output))


if __name__ == '__main__':
    main()
