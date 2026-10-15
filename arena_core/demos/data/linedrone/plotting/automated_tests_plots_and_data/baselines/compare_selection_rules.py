"""Offline comparison of the rules selecting a trajectory of a front from the risks (no planning).

Rules (plot_risk_sensitivity.selector):
    - rank: Eq. 11 (initial weights --k0) + adaptive_voting_algorithm::getBetterCandidateIndex (current ARENA)
    - tchebycheff: Eq. 11 + augmented weighted Tchebycheff
    - weights_map: risks mapped to weights spanning the simplex + augmented weighted Tchebycheff (method change)
    - aspiration: risks mapped to aspiration levels + achievement scalarizing function (method change)

1. Single fronts (<planner>_front/solutions.csv, the costs of the voting): number of solutions selected along every risk
   swept alone, monotonicity of the clearance, choice for joint risks, and reachability (share of the non-dominated
   solutions selected by at least one risk combination of the grid (battery, wind, localization) in [0, 1]^3, step 0.1).
2. Existing tests (report_3d_*_pareto_front.csv, the safe solutions of every planning with their reported costs):
    - reachability on every front of --reachability-tests (e.g. the 50 plannings of arena_default)
    - risk sweep tests (--risk-sweeps): every rule replayed on the front of every planning of the sweep, with the risks of
      this planning. Mean reported costs of the solution selected by every rule per risk level, next to the costs of
      the solution chosen by the planner during the test (rank + Eq. 11 on the costs of the optimizer).

Usage:
    python3 compare_selection_rules.py [--map CL_map_3] [--k0 0.25 0.5 0.25] [--w0 0.333 0.333 0.333]
        [--safety-demand or|max|eq11] [--risk-sweeps arena_risk_sweep_k025 ...] [--reachability-tests arena_default ...]
"""

import argparse
import csv
import glob
import itertools
import os
from collections import defaultdict

import numpy as np

from plot_risk_sensitivity import HERE, load_front, selector, sweep_front, vote_rank

RULES = ['rank', 'tchebycheff', 'weights_map', 'eq11_exp', 'aspiration']
GRID = np.round(np.arange(0.0, 1.0 + 1e-9, 0.1), 6)
RISK_COLUMNS = [('battery', 'Baterry risk'), ('wind', 'Wind risk'), ('localization', 'Location risk')]
JOINT = [(1.0, 1.0, 0.0), (0.5, 0.5, 0.0), (0.0, 1.0, 1.0), (0.5, 0.0, 0.5)]  # (battery, wind, localization)


def non_dominated(costs):
    """Indices of the candidates no other candidate dominates."""
    return [i for i in range(len(costs))
            if not any(np.all(costs[j] <= costs[i]) and np.any(costs[j] < costs[i]) for j in range(len(costs)))]


def reachability(costs, select):
    """Share of the non-dominated candidates selected by at least one risk combination of the grid."""
    pareto = set(non_dominated(costs))
    if len(costs) < 2:
        return 1.0
    reached = {select(costs, b, w, l) for b, w, l in itertools.product(GRID, GRID, GRID)}
    return len(reached & pareto) / len(pareto)


def load_pareto_fronts(folder):
    """Safe solutions (time, safety, energy reported costs) of every planning of the tests of a folder: {(file, id): costs}."""
    fronts = defaultdict(list)
    for path in sorted(glob.glob(os.path.join(folder, 'report_3d_*_pareto_front.csv'))):
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                if int(float(row['Safe'])) == 1:
                    fronts[(path, row['Id'])].append([float(row['Time cost']), float(row['Security cost']),
                                                      float(row['Energy cost'])])
    return {key: np.array(costs) for key, costs in fronts.items()}


def load_sweep_runs(folder):
    """Risks and chosen costs of every planning of the risk sweep tests of a folder: {(pareto file, id): row}."""
    runs = {}
    for path in sorted(glob.glob(os.path.join(folder, 'report_3d_*.csv'))):
        if path.endswith('_pareto_front.csv'):
            continue
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                runs[(path.replace('.csv', '_pareto_front.csv'), row['Id'])] = {
                    'risks': tuple(float(row[c]) for _, c in RISK_COLUMNS),
                    'weights': np.array([float(row['Time coefficient']), float(row['Security coefficient']),
                                         float(row['Energy coefficient'])]),
                    'chosen': [float(row['Chosen time cost']), float(row['Chosen security cost']),
                               float(row['Chosen energy cost'])]}
    return runs


def is_monotone(values, increasing):
    steps = np.diff(values)
    return bool(np.all(steps >= -1e-9)) if increasing else bool(np.all(steps <= 1e-9))


def single_fronts(base, selectors):
    print('=== Single fronts (solutions.csv, costs of the voting)')
    for name in ['arena_front', 'spline_nsga2_front', 'moar3d_sweep_front']:
        front = load_front(os.path.join(base, name))
        if front is None:
            continue
        vote, metrics = front
        print('\n{}: {} safe solutions, {} non-dominated'.format(name, len(vote), len(non_dominated(vote))))
        print('  {:<12} {:>9} {:>22} {:>22} {:>22}   {}'.format(
            'rule', 'reachable', 'wind: n / clearance', 'battery: n / duration', 'loc.: n / clearance',
            'joint (b, w, l) -> clearance m / duration s'))
        for rule, select in selectors.items():
            rows = sweep_front(front, select)
            cells = []
            for (_, column), metric, increasing in [(RISK_COLUMNS[1], 'clearance', True),
                                                    (RISK_COLUMNS[0], 'duration', False),
                                                    (RISK_COLUMNS[2], 'clearance', True)]:
                others = [c for _, c in RISK_COLUMNS if c != column]
                # One row per level (the null level is also in the sweeps of the other risks)
                swept = sorted({r[column]: r for r in rows if all(r[o] == 0.0 for o in others)}.values(),
                               key=lambda r: r[column])
                values = [r[metric] for r in swept]
                cells.append('{:>2} / {:5.1f}->{:5.1f}{}'.format(
                    len({r['solution'] for r in swept}), values[0], values[-1],
                    ' ' if is_monotone(values, increasing) else '*'))
            joint = ['({:g},{:g},{:g}) {:.1f}/{:.0f}'.format(b, w, l, metrics[select(vote, b, w, l)]['clearance'],
                                                             metrics[select(vote, b, w, l)]['duration'])
                     for b, w, l in JOINT]
            print('  {:<12} {:>8.0f}% {:>22} {:>22} {:>22}   {}'.format(
                rule, 100 * reachability(vote, select), *cells, ', '.join(joint)))
    print('  (* the metric is not monotone along the risk)')


def reachability_tests(base, folders, selectors):
    print('\n=== Reachability on the fronts of existing tests (reported costs of the safe solutions)')
    for folder in folders:
        fronts = load_pareto_fronts(os.path.join(base, folder))
        if not fronts:
            print('{}: no pareto front report'.format(folder))
            continue
        sizes = [len(non_dominated(c)) for c in fronts.values()]
        print('{}: {} plannings, {:.0f} non-dominated solutions on average'.format(folder, len(fronts), np.mean(sizes)))
        for rule, select in selectors.items():
            shares = [reachability(costs, select) for costs in fronts.values()]
            print('  {:<12} {:5.0f}% reachable (min {:.0f}%, max {:.0f}%)'.format(
                rule, 100 * np.mean(shares), 100 * np.min(shares), 100 * np.max(shares)))


def risk_sweeps(base, folders, selectors):
    for folder in folders:
        runs = load_sweep_runs(os.path.join(base, folder))
        fronts = load_pareto_fronts(os.path.join(base, folder))
        keys = [k for k in runs if k in fronts and len(fronts[k]) > 0]
        if not keys:
            print('\n{}: no risk sweep report'.format(folder))
            continue
        # The planner chose with the costs of the optimizer, the fronts hold the reported costs: the choice of the test is
        # found in the front by its reported costs
        matches = sum(int(np.any(np.all(np.isclose(fronts[k], runs[k]['chosen'], rtol=1e-6), axis=1))) for k in keys)
        replay = sum(int(np.allclose(fronts[k][vote_rank(fronts[k], runs[k]['weights'])], runs[k]['chosen'],
                                     rtol=1e-6)) for k in keys)
        print('\n=== Risk sweep {}: {} plannings, chosen solution found in its front for {}, rank replay on the '
              'reported costs = test choice for {}'.format(folder, len(keys), matches, replay))
        print('Mean reported costs (time s / safety / energy kJ) of the selected solution per risk level, every rule on '
              'the front of every planning:')
        print('  {:<13} {:>5} {:>24}'.format('risk', 'level', 'test (planner)') +
              ''.join('{:>24}'.format(rule) for rule in selectors))
        for name, column in [RISK_COLUMNS[1], RISK_COLUMNS[0], RISK_COLUMNS[2]]:
            index = [c for _, c in RISK_COLUMNS].index(column)
            for level in GRID:
                level_keys = [k for k in keys if runs[k]['risks'][index] == level
                              and all(r == 0.0 for i, r in enumerate(runs[k]['risks']) if i != index)]
                if not level_keys:
                    continue
                cells = [np.mean([runs[k]['chosen'] for k in level_keys], axis=0)]
                for select in selectors.values():
                    cells.append(np.mean([fronts[k][select(fronts[k], *runs[k]['risks'])] for k in level_keys], axis=0))
                print('  {:<13} {:>5.1f}'.format(name, level) + ''.join(
                    '{:>10.1f} /{:>5.3f} /{:>5.0f}'.format(c[0], c[1], c[2] * 1e-3) for c in cells))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', default='CL_map_3')
    parser.add_argument('--k0', nargs=3, type=float, default=[0.25, 0.5, 0.25], help='Initial weights of Eq. 11')
    parser.add_argument('--w0', nargs=3, type=float, default=[1/3, 1/3, 1/3],
                        help='Nominal weights of weights_map and aspiration')
    parser.add_argument('--safety-demand', choices=['or', 'max', 'eq11'], default='or')
    parser.add_argument('--battery-corner', nargs=3, type=float, default=[0.5, 0.0, 0.5])
    parser.add_argument('--risk-sweeps', nargs='*', default=['arena_risk_sweep_k025', 'arena_risk_sweep'])
    parser.add_argument('--reachability-tests', nargs='*', default=['arena_default'])
    args = parser.parse_args()

    base = os.path.join(HERE, args.map)
    selectors = {rule: selector(rule, args.k0, args.w0, args.safety_demand, args.battery_corner) for rule in RULES}
    print('Eq. 11 k0 = {}, w0 = {}, safety demand = {}, battery corner = {}'.format(
        args.k0, np.round(args.w0, 3).tolist(), args.safety_demand, args.battery_corner))
    single_fronts(base, selectors)
    reachability_tests(base, args.reachability_tests, selectors)
    risk_sweeps(base, args.risk_sweeps, selectors)


if __name__ == '__main__':
    main()
