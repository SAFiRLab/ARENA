"""Sensitivity of the chosen trajectory to the risks, ARENA compared to the benchmark planners (Fig. 4 of the paper).

Grid of 3 x 3 panels: one column per risk (wind, battery, localization), swept from 0 to 1 with the other risks null, and
one row per metric of the trajectory chosen by the planner:
    - average distance to the obstacles (m): mean over the samples of the published trajectory (evaluation sample size)
      of the distance to the closest occupied voxel of the inflated octomap (testbench_node::getClosestObstacleDistance)
    - path duration (s): time cost of the trajectory (ARENA time objective, with the planned speeds)
    - energy (kJ): energy cost of the trajectory (ARENA energy model)
The costs are the ones reported by the planners, evaluated with the evaluation sample size of the tests (250 samples).

Data (one folder per planner in baselines/<map>/, written by the testbench):
    - ARENA (black dots) and MOAR-3D (hollow squares): risk sweep tests (arena_risk_sweep, moar3d_risk_sweep), the
      cost weights follow the risks (Eq. 11 of ARENA = Eq. 9 of MOAR) and the trajectory is chosen by each planner
    - planners that don't adapt to the risks, shaded band from the minimum to the maximum of the metric:
        * A* (line, deterministic), RRT* 0.75 s and 5 s (hatched bands): their planning tests (astar, rrt_star_short,
          rrt_star_long), the chosen trajectory doesn't depend on the weights
        * Spline NSGA-II and MOAR-3D weight sweep: the trajectories chosen from their front by the adaptive voting
          algorithm during their risk sweep tests (spline_nsga2_risk_sweep, moar3d_sweep_risk_sweep), for the risk of
          the column
    - optimal trajectories (dashed lines): best trajectory of every objective over the runs of ARENA with the
      hyperparameters of the reference front and the cost weights 1/0/0, 0/1/0 and 0/0/1 (optimal_time,
      optimal_safety, optimal_energy): metrics of the trajectory with the lowest time, safety or energy cost

--suffix selects other risk sweep tests, e.g. _k025 for the folders arena_risk_sweep_k025, ... run with other initial
weights of Eq. 11 (a planner without such a folder keeps its default risk sweep, with a warning).

--fronts replays the voting on single fronts instead of risk sweep tests: <planner>_front/solutions.csv holds every safe
solution of one planning (optimization.export_solutions_file) with the costs used by its voting, its reported costs and
the clearance of its reported trajectory. For every risk level, the risks select one solution with the rule --voting, the
same front for every risk:
    - rank: weights of Eq. 11 (initial weights --k0), adaptive_voting_algorithm::getBetterCandidateIndex (ARENA)
    - tchebycheff: weights of Eq. 11, augmented weighted Tchebycheff (getTchebycheffCandidateIndex)
    - weights_map (method change under study): the risks map to weights spanning the simplex, the safety demand s
      (--safety-demand, default s = 1 - (1 - W)(1 - L)) pulls the weights to (0, 1, 0), the battery risk b to --battery-corner,
      w = (1 - s)(1 - b) w0 + s (0, 1, 0) + (1 - s) b corner, augmented weighted Tchebycheff
    - eq11_exp (method change under study): the influences of the risks on the weights of Eq. 11 (EQ11_INFLUENCE, wind
      on energy 0.25 instead of 0.5) in exponential form, k_j = w0_j exp(gamma sum_r a_jr r) normalized (Eq. 11 is its
      first order), every risk always changes every weight, augmented weighted Tchebycheff
    - aspiration (method change under study): equal weights w0, the risks set aspiration levels in the costs normalized
      on the front (0: best, 1: worst), z = (1 - b, 1 - s, 1 - b), achievement scalarizing function (Wierzbicki)
      max_j w0_j (n_ij - z_j) + rho sum_j w0_j (n_ij - z_j) ARENA (arena_front) is drawn as one point per risk level, Spline NSGA-II and MOAR-3D weight
sweep (spline_nsga2_front, moar3d_sweep_front) as the band of their selected solutions, MOAR-3D from --moar-risk-sweep.

--sweep-fronts replays the rule --voting on the fronts of the existing risk sweep tests instead of one front: every
planning of the risk sweeps of a planner (<planner>_risk_sweep<suffix> for every suffix of SWEEP_SUFFIXES, the cost
weights and the voting of the test don't change the front) has its safe solutions in *_pareto_front.csv, the rule selects
one of them with the risks of this planning. ARENA is drawn as the median (dot) and the interquartile range (bar) of the
selected solutions per risk level, Spline NSGA-II and MOAR-3D weight sweep as the band of their selected solutions. The
fronts only hold the costs, the first row is then the safety cost of the selected solution instead of its clearance.

--reference-front replays the rule --voting on the reference front of the map (estimated Pareto front of the reference
runs, hyperparameter_variations/<map>/reference_front/pareto_front_3d_estimated_front.csv): one point per risk level like
--fronts, the first row is the safety cost. The reference runs are evaluated with their own sample size (50 samples for
CL_map_3), not the 250 samples of the baselines: the baselines aren't drawn, the optimal lines are the best solution of
every objective of the reference front.

Options of the replays:
    --every-level (with --sweep-fronts): every front of the risk sweep tests is replayed at every risk level of --step
      instead of the risks of its planning (the risks don't change the front, only the choice)
    --no-baselines: ARENA only
    --optimal-fronts (with --sweep-fronts): optimal lines from the replayed fronts, median over the fronts of the metrics
      of the best solution of every objective of each front (what the selection can reach at most)
    --optimal-reference [<csv>]: optimal lines from the reference front of the map (best solution of every objective)
      instead of the optimal_* tests. The reference runs are evaluated with 50 samples, the tests with 250: same
      durations, energies about 1 % lower, safety costs about 1.4 times lower (arena_front/solutions.csv)

Usage:
    python3 plot_risk_sensitivity.py [--reference-front [<csv>]|--sweep-fronts|--fronts [--k0 0.25 0.5 0.25] [--voting rank|tchebycheff|weights_map|eq11_exp|aspiration] [--gamma 10] [--influence eq11|scaled] [--risk-transform linear|hazard]
        [--w0 0.333 0.333 0.333] [--safety-demand or|max|eq11] [--background 0.5] [--step 0.1] [--battery-corner 0.5 0 0.5]] [--map CL_map_3]
        [--suffix _k025] [--output <prefix>] [--size 3.5 3.9] [--dpi 400]
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'hyperparameter_variations'))
from plot_computation_analysis import PAPER_STYLE  # noqa: E402

RISKS = [('Wind risk', 'Wind risk'), ('Baterry risk', 'Battery risk'), ('Location risk', 'Localization risk')]
METRICS = [('clearance', 'Avg. obstacle\nclearance (m)'), ('duration', 'Path\nduration (s)'), ('energy', 'Energy (kJ)')]
COST_METRICS = [('safety', 'Safety cost'), ('duration', 'Path\nduration (s)'), ('energy', 'Energy (kJ)')]  # --sweep-fronts
SWEEP_SUFFIXES = ['', '_k025', '_tcheby', '_tcheby_k025']  # Risk sweep tests of a planner pooled by --sweep-fronts

# Optimal trajectories (dashed lines), colors of the original figure, the line style is a second cue
OPTIMAL = [('optimal_time', 0, 'Time optimal', '#e34948', (0, (4, 2))),
           ('optimal_safety', 1, 'Safety optimal', '#2a78d6', (0, (1, 1.5))),
           ('optimal_energy', 2, 'Energy optimal', '#008300', (0, (5, 1.5, 1, 1.5)))]

# Planners that don't adapt to the risks: (folder, label, color, kind, hatch). Three hues validated all-pairs under
# colorblind simulation, the single-path planners share one hue told apart by the mark (line) and the hatch.
BANDS = [('moar3d_sweep_risk_sweep', 'MOAR-3D (weight sweep)', '#eb6834', 'risk', None),
         ('spline_nsga2_risk_sweep', 'Spline NSGA-II', '#1baf7a', 'risk', None),
         ('rrt_star_short', 'RRT* (0.75 s)', '#4a3aa7', 'planning', '////'),
         ('rrt_star_long', 'RRT* (5 s)', '#4a3aa7', 'planning', '\\\\\\\\'),
         ('astar', 'A*', '#4a3aa7', 'line', None)]

# Multi-objective baselines added for the revision (--extra-bands): their fronts (<x>_front/solutions.csv with --fronts).
# Five hues: min OKLab dE 20 (normal vision), 7.6 under deuteranopia, the hatches tell A*pex and PO-RRT* apart
EXTRA_BANDS = [('apex_risk_sweep', 'A*pex', '#ad1457', 'risk', '..'),
               ('po_rrt_star_risk_sweep', 'PO-RRT*', '#7a5c00', 'risk', 'xx')]

ARENA_STYLE = {'color': '#0b0b0b', 'marker': 'o', 's': 6, 'linewidths': 0}
MOAR_STYLE = {'facecolors': 'none', 'edgecolors': '#52514e', 'marker': 's', 's': 10, 'linewidths': 0.6}
BAND_ALPHA = 0.14
HATCH_LINE_WIDTH = 0.35
Y_MARGIN = 0.12  # Fraction of the range of the points and lines added above and below
X_JITTER = 0.012  # Points of the same risk level drawn side by side


def report_files(folder):
    return sorted(f for f in glob.glob(os.path.join(folder, 'report_3d_*.csv'))
                  if not f.endswith('_pareto_front.csv') and not f.endswith('_solutions.csv'))


def cost_key(time, safety, energy):
    """Key of a solution by its reported costs (written with 6 decimals in the reports and in the exports)."""
    return round(float(time), 4), round(float(safety), 5), round(float(energy), 2)


def load_clearances(folders):
    """Average clearance of every exported solution, by reported costs: report_3d_*_solutions.csv of the folders, every
    solution of the reports with its reported costs and the average distance of its trajectory to the inflated octomap
    (optimization.export_solutions_file of the planner)."""
    table = {}
    for folder in folders:
        for path in sorted(glob.glob(os.path.join(folder, 'report_3d_*_solutions.csv'))):
            with open(path, newline='') as f:
                for row in csv.DictReader(f):
                    table[cost_key(row['Time cost'], row['Security cost'], row['Energy cost'])] = \
                        float(row['Average Closest Obstacle Distance'])
    return table


def clearance_of(clearances, costs, missing):
    """Clearance of a solution from its costs (time, safety, energy), NaN and counted in missing[0] if not exported."""
    value = clearances.get(cost_key(*costs[:3]))
    if value is None:
        missing[0] += 1
        return float('nan')
    return value


def load_risk_sweep(folder):
    """Rows of the risks report: risks (battery, wind, location) and the metrics of the chosen trajectory."""
    rows = []
    for path in report_files(folder):
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                rows.append({
                    'Baterry risk': float(row['Baterry risk']), 'Wind risk': float(row['Wind risk']),
                    'Location risk': float(row['Location risk']),
                    'clearance': float(row['Average Cloest Obstacle Distance']),
                    'safety': float(row['Chosen security cost']),
                    'duration': float(row['Chosen time cost']),
                    'energy': float(row['Chosen energy cost']) * 1.0e-3,
                })
    return rows


def load_planning(folder):
    """Metrics of the chosen trajectory of every feasible run of a planning report (hyperparameter tests)."""
    runs = {}
    for path in report_files(folder):
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                if int(float(row['Feasible'])) != 1:
                    continue
                key = (path, row['Id'])
                run = runs.setdefault(key, {'distances': [], 'duration': float(row['Chosen time cost']),
                                            'energy': float(row['Chosen energy cost']) * 1.0e-3,
                                            'costs': [float(row['Chosen time cost']), float(row['Chosen security cost']),
                                                      float(row['Chosen energy cost'])]})
                run['distances'].append(float(row['Closest Obstacle Distance']))
    for run in runs.values():
        run['clearance'] = float(np.mean(run['distances']))
        run['safety'] = run['costs'][1]
    return list(runs.values())


def load_front(folder):
    """Safe solutions of one planning exported by the planner (solutions.csv)."""
    path = os.path.join(folder, 'solutions.csv')
    if not os.path.isfile(path):
        return None
    with open(path, newline='') as f:
        rows = [r for r in csv.DictReader(f) if r['Planning'] == '0']
    vote = np.array([[float(r['Vote time cost']), float(r['Vote security cost']), float(r['Vote energy cost'])] for r in rows])
    metrics = [{'clearance': float(r['Average Closest Obstacle Distance']), 'safety': float(r['Security cost']),
                'duration': float(r['Time cost']),
                'energy': float(r['Energy cost']) * 1.0e-3} for r in rows]
    return vote, metrics


def eq11(battery, wind, location, k0):
    """Weights (time, safety, energy) of Eq. 11 from the risks, without the communication risk (testbench_node)."""
    s = 0.5 * wind + 0.25 * location - battery
    return np.array([k0[0] * (1.0 - s), k0[1] * (1.0 + s), k0[2] * (1.0 + 0.5 * wind + 0.5 * battery)])


def vote_rank(costs, weights):
    """adaptive_voting_algorithm::getBetterCandidateIndex: rank = number of strictly better candidates, normalized by
    the largest rank of the objective, weighted sum, first index of the minimum."""
    ranks = (costs[None, :, :] < costs[:, None, :]).sum(axis=1).astype(float)
    ranks /= np.where(ranks.max(axis=0) > 0, ranks.max(axis=0), 1.0)
    return int(np.argmin(ranks @ weights))


def normalize(costs):
    """Min-max normalization of every objective over the candidates (0: best, 1: worst)."""
    span = costs.max(axis=0) - costs.min(axis=0)
    return (costs - costs.min(axis=0)) / np.where(span > 0, span, 1.0)


def vote_tchebycheff(costs, weights, rho=1.0e-3):
    """adaptive_voting_algorithm::getTchebycheffCandidateIndex."""
    weighted = normalize(costs) * weights
    return int(np.argmin(weighted.max(axis=1) + rho * weighted.sum(axis=1)))


def vote_aspiration(costs, aspiration, weights, rho=1.0e-3):
    """Achievement scalarizing function: weighted Tchebycheff distance to the aspiration point in normalized costs."""
    weighted = (normalize(costs) - aspiration) * weights
    return int(np.argmin(weighted.max(axis=1) + rho * weighted.sum(axis=1)))


def safety_demand(wind, location, rule):
    """Safety demand s in [0, 1] of the wind and localization risks (no communication risk in the tests):
        - or: probabilistic or, s = 1 - (1 - W)(1 - L), every risk raises s unless another one is already at 1 and one
          risk alone at 1 asks for the safest trajectory
        - max: only the largest risk counts
        - eq11: weights of the safety term of Eq. 11"""
    if rule == 'or':
        return 1.0 - (1.0 - wind) * (1.0 - location)
    return max(wind, location) if rule == 'max' else min(1.0, 0.5 * wind + 0.25 * location)


def mapped_weights(battery, wind, location, w0, rule='or', corner=(0.5, 0.0, 0.5)):
    """Weights (time, safety, energy) spanning the simplex: s pulls them to (0, 1, 0), the battery risk to corner."""
    s = safety_demand(wind, location, rule)
    return ((1.0 - s) * (1.0 - battery) * np.asarray(w0) + s * np.array([0.0, 1.0, 0.0])
            + (1.0 - s) * battery * np.asarray(corner))


def aspiration_point(battery, wind, location, rule='or'):
    """Aspiration levels (time, safety, energy) in the normalized costs: the safety demand lowers the safety level, the
    battery risk the time and energy levels."""
    s = safety_demand(wind, location, rule)
    return np.array([1.0 - battery, 1.0 - s, 1.0 - battery])


# Influence of the risks (wind, communication, localization, battery) on the weights (time, safety, energy): the terms of
# Eq. 11 (k_T = k0_T (1 - s), k_S = k0_S (1 + s), k_E = k0_E (1 + 0.5 W + 0.5 B), s = 0.5 W + 0.25 C + 0.25 L - B) with
# the influence of the wind on energy lowered to 0.25 (with 0.5, the wind pushes the safety and energy weights equally and
# doesn't change the choice: the safest trajectories need the most energy)
EQ11_INFLUENCE = np.array([[-0.5, -0.25, -0.25, 1.0],
                           [0.5, 0.25, 0.25, -1.0],
                           [0.25, 0.0, 0.0, 0.5]])


# Same influences with the column of every risk scaled to a largest magnitude of 1 (--influence scaled): the signs and the
# split of every risk between the objectives stay those of Eq. 11, every risk alone moves the weights as far as the others
EQ11_INFLUENCE_SCALED = EQ11_INFLUENCE / np.abs(EQ11_INFLUENCE).max(axis=0)


def eq11_exp_weights(battery, wind, location, w0, gamma, communication=0.0, influence=EQ11_INFLUENCE,
                     transform='linear'):
    """Weights (time, safety, energy) of the exponential form of Eq. 11: w0_j exp(gamma sum_r a_jr h(r)), normalized.

    h(r) = r (linear) or h(r) = -ln(1 - r) (hazard): w0_j prod_r (1 - r)^(-gamma a_jr), equal to the linear form at the
    first order (Eq. 11 for gamma = 1) and infinite at r = 1, a risk at 1 gives all the weight to the objective it pushes
    the most (the selection reaches the extreme of the front)."""
    risks = np.array([wind, communication, location, battery])
    if transform == 'hazard':
        risks = -np.log1p(-np.minimum(risks, 1.0 - 1.0e-12))
    exponents = gamma * influence @ risks
    weights = np.asarray(w0) * np.exp(exponents - exponents.max())
    return weights / weights.sum()


def selector(voting, k0=(0.25, 0.5, 0.25), w0=(1/3, 1/3, 1/3), rule='or', corner=(0.5, 0.0, 0.5), gamma=10.0,
             influence='eq11', transform='linear'):
    """Function (costs, battery, wind, location) -> index of the selected candidate for the rule --voting."""
    if voting == 'rank':
        return lambda costs, b, w, l: vote_rank(costs, eq11(b, w, l, k0))
    if voting == 'tchebycheff':
        return lambda costs, b, w, l: vote_tchebycheff(costs, eq11(b, w, l, k0))
    if voting == 'weights_map':
        return lambda costs, b, w, l: vote_tchebycheff(costs, mapped_weights(b, w, l, w0, rule, corner))
    if voting == 'eq11_exp':
        matrix = EQ11_INFLUENCE_SCALED if influence == 'scaled' else EQ11_INFLUENCE
        return lambda costs, b, w, l: vote_tchebycheff(costs, eq11_exp_weights(b, w, l, w0, gamma, influence=matrix,
                                                                                  transform=transform))
    if voting == 'aspiration':
        return lambda costs, b, w, l: vote_aspiration(costs, aspiration_point(b, w, l, rule), np.asarray(w0))
    raise ValueError(voting)


RISK_ORDER = ['Baterry risk', 'Wind risk', 'Location risk']  # Order of the risks of select()


def background_risks(column, level, background):
    """Risks (battery, wind, location) of the sweep of the risk column at level, the other risks at background."""
    risks = [background] * 3
    risks[RISK_ORDER.index(column)] = level
    return risks


def sweep_front(front, select, step=0.1, background=0.0):
    """Rows like a risk sweep test: every risk from 0 to 1 by step with the others at background, the solution voted on
    the front. The rows hold the swept risk only (column), like the risk sweep tests."""
    vote, metrics = front
    rows = []
    for column in RISK_ORDER:
        for level in np.round(np.arange(0.0, 1.0 + 1e-9, step), 6):
            chosen = select(vote, *background_risks(column, level, background))
            row = {r: 0.0 for r in RISK_ORDER}
            row.update({column: level, 'column': column, 'solution': chosen})
            rows.append(dict(row, **metrics[chosen]))
    return rows


def load_reference_front(path, clearances=None):
    """Costs (time, safety, energy) of the estimated Pareto front of the reference runs and their metrics, with the
    average clearance of every solution if clearances (load_clearances) is given."""
    with open(path, newline='') as f:
        costs = np.array([[float(r['Time cost']), float(r['Security cost']), float(r['Energy cost'])]
                          for r in csv.DictReader(f)])
    metrics = [{'safety': c[1], 'duration': c[0], 'energy': c[2] * 1.0e-3, 'costs': list(c)} for c in costs]
    if clearances is not None:
        missing = [0]
        for m, c in zip(metrics, costs):
            m['clearance'] = clearance_of(clearances, c, missing)
        if missing[0]:
            print('Warning: {} of the {} solutions of {} have no exported clearance'.format(missing[0], len(costs), path),
                  file=sys.stderr)
    return costs, metrics


def load_sweep_fronts(folders, clearances=None):
    """Risks (battery, wind, location) and safe solutions (time, safety, energy reported costs, and the average clearance
    if clearances (load_clearances) is given) of every planning of the risk sweep tests of the folders."""
    plannings = []
    missing = [0]
    for folder in folders:
        for path in report_files(folder):
            fronts, seen = {}, set()
            with open(path.replace('.csv', '_pareto_front.csv'), newline='') as f:
                for row in csv.DictReader(f):
                    if int(float(row['Safe'])) == 1:
                        costs = [float(row['Time cost']), float(row['Security cost']), float(row['Energy cost'])]
                        if clearances is not None:
                            costs.append(clearance_of(clearances, costs, missing))
                        fronts.setdefault(row['Id'], []).append(costs)
            with open(path, newline='') as f:
                for row in csv.DictReader(f):
                    # One planning per Id (the reports of the hyperparameter tests have a row per trajectory sample)
                    if row['Id'] in fronts and row['Id'] not in seen:
                        seen.add(row['Id'])
                        # The hyperparameter tests have no risk columns: plannings without risk
                        plannings.append(((float(row.get('Baterry risk', 0.0)), float(row.get('Wind risk', 0.0)),
                                           float(row.get('Location risk', 0.0))), np.array(fronts[row['Id']])))
    if missing[0]:
        print('Warning: {} solutions of the fronts have no exported clearance'.format(missing[0]), file=sys.stderr)
    return plannings


def replay_sweep_fronts(plannings, select, background=0.0):
    """Rows like a risk sweep test: the solution selected by the rule on the front of every planning, with the risk swept
    by this planning and the other risks at background (a planning without risk belongs to the sweep of every risk)."""
    rows = []
    for risks, costs in plannings:
        swept = [c for c, r in zip(RISK_ORDER, risks) if r != 0.0] or RISK_ORDER
        for column in swept:
            level = risks[RISK_ORDER.index(column)]
            chosen = costs[select(costs[:, :3], *background_risks(column, level, background))]
            row = {r: 0.0 for r in RISK_ORDER}
            row.update({column: level, 'column': column, 'safety': chosen[1], 'duration': chosen[0],
                        'energy': chosen[2] * 1.0e-3})
            if len(chosen) > 3:
                row['clearance'] = chosen[3]
            rows.append(row)
    return rows


def sweep_folders(base, name):
    """Existing risk sweep folders of a planner (name + every suffix of SWEEP_SUFFIXES)."""
    return [os.path.join(base, name + suffix) for suffix in SWEEP_SUFFIXES
            if report_files(os.path.join(base, name + suffix))]


def quartiles(rows, risk, metric):
    """Levels of the risk and the 25 %, 50 % and 75 % quantiles of the metric per level."""
    levels = sorted({r[risk] for r in rows})
    values = [np.percentile([r[metric] for r in rows if r[risk] == level], [25, 50, 75]) for level in levels]
    return np.array(levels), np.array(values).reshape(-1, 3)


def rows_of_risk(rows, risk):
    """Rows of the sweep of a risk: the rows of this column (replays) or the other risks are null (tests)."""
    others = [r for r, _ in RISKS if r != risk]
    return [r for r in rows if r['column'] == risk] if rows and 'column' in rows[0] else \
        [r for r in rows if all(abs(r[o]) < 1e-9 for o in others)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', default='CL_map_3', help='Folder of the map in baselines/')
    parser.add_argument('--suffix', default='', help='Suffix of the risk sweep folders (e.g. _k025)')
    parser.add_argument('--fronts', action='store_true', help='Replay the voting on the single fronts (<planner>_front)')
    parser.add_argument('--reference-front', nargs='?', const='', default=None, metavar='CSV',
                        help='Replay the voting on the reference front (default: estimated front of the reference runs '
                             'of the map in hyperparameter_variations)')
    parser.add_argument('--sweep-fronts', action='store_true',
                        help='Replay the voting on the fronts of every planning of the existing risk sweep tests')
    parser.add_argument('--k0', nargs=3, type=float, default=[0.25, 0.5, 0.25], metavar=('TIME', 'SAFETY', 'ENERGY'),
                        help='Initial weights of Eq. 11 with --fronts')
    parser.add_argument('--voting', choices=['rank', 'tchebycheff', 'weights_map', 'eq11_exp', 'aspiration'], default='rank',
                        help='Selection rule with --fronts')
    parser.add_argument('--gamma', type=float, default=10.0, help='Gain of the exponential form of Eq. 11 (eq11_exp)')
    parser.add_argument('--risk-transform', choices=['linear', 'hazard'], default='linear',
                        help='eq11_exp: risk r (linear) or -ln(1 - r) (hazard, a risk at 1 reaches the extreme)')
    parser.add_argument('--influence', choices=['eq11', 'scaled'], default='eq11',
                        help='eq11_exp: influences of Eq. 11 or scaled per risk (EQ11_INFLUENCE_SCALED)')
    parser.add_argument('--w0', nargs=3, type=float, default=[1/3, 1/3, 1/3], metavar=('TIME', 'SAFETY', 'ENERGY'),
                        help='Nominal weights of weights_map and aspiration')
    parser.add_argument('--step', type=float, default=0.1,
                        help='Step of the risk levels of the replays on single fronts (--fronts, --reference-front, '
                             '--sweep-fronts --every-level)')
    parser.add_argument('--every-level', action='store_true',
                        help='--sweep-fronts: replay every front at every risk level of --step')
    parser.add_argument('--no-baselines', action='store_true', help='Draw ARENA only')
    parser.add_argument('--optimal-fronts', action='store_true',
                        help='--sweep-fronts: optimal lines from the replayed fronts (median of the best solution of every '
                             'objective of each front)')
    parser.add_argument('--optimal-reference', nargs='?', const='', default=None, metavar='CSV',
                        help='Optimal lines from the reference front (default: estimated front of the reference runs '
                             'of the map in hyperparameter_variations)')
    parser.add_argument('--background', type=float, default=0.0,
                        help='Level of the risks not swept in a column (replays: --fronts, --sweep-fronts, '
                             '--reference-front)')
    parser.add_argument('--safety-demand', choices=['or', 'max', 'eq11'], default='or',
                        help='Safety demand of the wind and localization risks (weights_map, aspiration)')
    parser.add_argument('--battery-corner', nargs=3, type=float, default=[0.5, 0.0, 0.5],
                        metavar=('TIME', 'SAFETY', 'ENERGY'), help='Weights asked by the battery risk (weights_map)')
    parser.add_argument('--rrt-short-label', default='RRT* (0.75 s)',
                        help='Label of the short RRT* (its time budget depends on the map, e.g. 0.9 s on seed_42)')
    parser.add_argument('--extra-bands', action='store_true',
                        help='Add the multi-objective baselines A*pex and PO-RRT* (EXTRA_BANDS) to the bands')
    parser.add_argument('--moar-risk-sweep', default='moar3d_risk_sweep_k025',
                        help='Risk sweep tests of MOAR-3D with --fronts (it plans a single path, it doesn\'t vote)')
    parser.add_argument('--clearance', action='store_true',
                        help='With --reference-front or --sweep-fronts: first row = average obstacle clearance of the '
                             'selected solution, from the exported solutions (report_3d_*_solutions.csv next to the '
                             'reports) instead of the safety cost')
    parser.add_argument('--fronts-folder',
                        help='With --sweep-fronts: folder of baselines/<map> whose plannings give the fronts (e.g. '
                             'default_params_rrt10) instead of the ARENA risk sweep tests')
    parser.add_argument('--output', help='Prefix of the figure files (default: baselines/<map>/risk_sensitivity<suffix>)')
    parser.add_argument('--size', nargs=2, type=float, default=[3.5, 3.9], metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--dpi', type=int, default=400)
    args = parser.parse_args()

    base = os.path.join(HERE, args.map)
    variant = '' if args.voting in ['rank', 'tchebycheff', 'eq11_exp'] or args.safety_demand == 'or' \
        else '_' + args.safety_demand
    variant += '_scaled' if args.voting == 'eq11_exp' and args.influence == 'scaled' else ''
    variant += '_hazard' if args.voting == 'eq11_exp' and args.risk_transform == 'hazard' else ''
    variant += '_g{:g}'.format(args.gamma) if args.voting == 'eq11_exp' and args.gamma != 10.0 else ''

    variant += '_bg{:g}'.format(args.background).replace('.', '') if args.background else ''
    front_name = '_front' + ('' if args.voting == 'rank' else '_' + args.voting) + variant
    if args.reference_front is not None:
        args.fronts, args.sweep_fronts = False, False
        output = args.output or os.path.join(base, 'risk_sensitivity_reference_' + args.voting + variant)
    elif args.sweep_fronts:
        args.fronts = False
        output = args.output or os.path.join(base, 'risk_sensitivity_sweeps_' + args.voting + variant)
    else:
        output = args.output or os.path.join(base, 'risk_sensitivity' + (front_name if args.fronts else args.suffix))
    metrics_of_rows = COST_METRICS if (args.sweep_fronts or args.reference_front is not None) and not args.clearance \
        else METRICS
    select = selector(args.voting, args.k0, args.w0, args.safety_demand, args.battery_corner, args.gamma,
                      args.influence, args.risk_transform)

    def risk_folder(name):
        folder = os.path.join(base, name + args.suffix)
        if args.suffix and not report_files(folder):
            print('Warning: no {} report, {} is used instead'.format(name + args.suffix, name), file=sys.stderr)
            folder = os.path.join(base, name)
        return folder

    reference_path = os.path.join(HERE, '..', 'hyperparameter_variations', args.map, 'reference_front',
                                  'pareto_front_3d_estimated_front.csv')
    if args.reference_front is not None:
        path = args.reference_front or reference_path
        front = load_reference_front(path, load_clearances([os.path.dirname(path)]) if args.clearance else None)
        arena = sweep_front(front, select, args.step, args.background)
        print('Reference front {}: {} solutions, {} selected over the risk sweep'.format(
            path, len(front[1]), len(set(r['solution'] for r in arena))))
        moar = []
    elif args.sweep_fronts:
        folders = [os.path.join(base, args.fronts_folder)] if args.fronts_folder else sweep_folders(base, 'arena_risk_sweep')
        plannings = load_sweep_fronts(folders, load_clearances(folders) if args.clearance else None)
        if args.every_level:
            arena = []
            for _, costs in plannings:
                metrics = [dict({'safety': c[1], 'duration': c[0], 'energy': c[2] * 1.0e-3},
                                **({'clearance': c[3]} if len(c) > 3 else {})) for c in costs]
                arena += sweep_front((costs[:, :3], metrics), select, args.step, args.background)
        else:
            arena = replay_sweep_fronts(plannings, select, args.background)
        print('ARENA: {} plannings of {}{}'.format(len(plannings), ', '.join(os.path.basename(f) for f in folders),
                                                  ', every front at every risk level' if args.every_level else ''))
        moar = load_risk_sweep(os.path.join(base, args.moar_risk_sweep))
    elif args.fronts:
        front = load_front(os.path.join(base, 'arena_front'))
        if front is None:
            sys.exit('No solutions.csv in {}'.format(os.path.join(base, 'arena_front')))
        arena = sweep_front(front, select, args.step, args.background)
        print('ARENA front: {} safe solutions, {} selected over the risk sweep'.format(
            len(front[1]), len(set(r['solution'] for r in arena))))
        moar = load_risk_sweep(os.path.join(base, args.moar_risk_sweep))
    else:
        arena = load_risk_sweep(risk_folder('arena_risk_sweep'))
        moar = load_risk_sweep(risk_folder('moar3d_risk_sweep'))
    if not arena:
        sys.exit('No ARENA risk sweep report in {}'.format(os.path.join(base, 'arena_risk_sweep')))
    if args.no_baselines:
        moar = []

    bands = []
    bands_list = [(f, args.rrt_short_label if f == 'rrt_star_short' else l, c, k, h) for f, l, c, k, h in BANDS]
    bands_list += EXTRA_BANDS if args.extra_bands else []
    for folder, label, color, kind, hatch in ([] if args.reference_front is not None or args.no_baselines else bands_list):
        if kind == 'risk' and args.sweep_fronts:
            folders = sweep_folders(base, folder)
            path = os.path.join(base, folder)
            planner_plannings = load_sweep_fronts(folders)
            data = replay_sweep_fronts(planner_plannings, select, args.background)
            if data:
                print('{}: {} plannings of {}'.format(label, len(planner_plannings),
                                                      ', '.join(os.path.basename(f) for f in folders)))
        elif kind == 'risk' and args.fronts:
            path = os.path.join(base, folder.replace('_risk_sweep', '_front'))
            planner_front = load_front(path)
            data = sweep_front(planner_front, select, args.step, args.background) if planner_front else []
            if data:
                print('{} front: {} safe solutions, {} selected over the risk sweep'.format(
                    label, len(planner_front[1]), len(set(r['solution'] for r in data))))
        else:
            path = risk_folder(folder) if kind == 'risk' else os.path.join(base, folder)
            data = load_risk_sweep(path) if kind == 'risk' else load_planning(path)
        if not data:
            print('Warning: no report for {} in {}, it is skipped'.format(label, path), file=sys.stderr)
            continue
        bands.append((label, color, kind, hatch, data))

    # Optimal trajectory of every objective: lowest cost of this objective over the runs
    optimal = []
    from_reference = args.reference_front is not None or args.optimal_reference is not None
    if args.sweep_fronts and args.optimal_fronts:  # Median over the fronts of the best solution of every objective
        from_reference = True
        for _, objective, label, color, style in OPTIMAL:
            bests = np.array([costs[np.argmin(costs[:, objective])] for _, costs in plannings])
            best = {'duration': float(np.median(bests[:, 0])), 'safety': float(np.median(bests[:, 1])),
                    'energy': float(np.median(bests[:, 2])) * 1.0e-3}
            if bests.shape[1] > 3:
                best['clearance'] = float(np.nanmedian(bests[:, 3]))
            optimal.append((label, color, style, best))
            print('{} (median of the fronts): duration {:.1f} s, safety cost {:.3f}, energy {:.1f} kJ'.format(
                label, best['duration'], best['safety'], best['energy']))
    elif from_reference:  # Best solution of every objective of the reference front
        reference = front if args.reference_front is not None else \
            load_reference_front(args.optimal_reference or reference_path)
        for _, objective, label, color, style in OPTIMAL:
            best = min(reference[1], key=lambda m: m['costs'][objective])
            optimal.append((label, color, style, best))
            print('{} (reference front): duration {:.1f} s, safety cost {:.3f}, energy {:.1f} kJ'.format(
                label, best['duration'], best['safety'], best['energy']))
    for folder, objective, label, color, style in ([] if from_reference else OPTIMAL):
        runs = load_planning(os.path.join(base, folder))
        if not runs:
            print('Warning: no report for {} in {}, it is skipped'.format(label, folder), file=sys.stderr)
            continue
        best = min(runs, key=lambda r: r['costs'][objective])
        optimal.append((label, color, style, best))
        print('{}: {} runs, best costs (time, safety, energy) {} -> clearance {:.2f} m, duration {:.1f} s, '
              'energy {:.1f} kJ'.format(label, len(runs), np.round(best['costs'], 3), best['clearance'],
                                        best['duration'], best['energy']))

    plt.rcParams.update(PAPER_STYLE)
    plt.rcParams['hatch.linewidth'] = HATCH_LINE_WIDTH
    # The y axis of every row is fitted to the points of the adaptive planners and to the optimal lines, the bands of the
    # other planners can go beyond it (they are clipped at the edge of the panel)
    core_values = [[] for _ in metrics_of_rows]
    band_limits = [[] for _ in metrics_of_rows]
    for row, (metric, _) in enumerate(metrics_of_rows):
        if args.sweep_fronts:  # Interquartile range of ARENA, the outliers can go beyond the panel
            for risk, _ in RISKS:
                core_values[row] += list(quartiles(rows_of_risk(arena, risk), risk, metric)[1][:, [0, 2]].ravel())
            core_values[row] += [r[metric] for r in moar] + [best[metric] for _, _, _, best in optimal]
        else:
            core_values[row] += [r[metric] for r in arena + moar] + [best[metric] for _, _, _, best in optimal]
    fig, axes = plt.subplots(len(metrics_of_rows), len(RISKS), figsize=args.size, sharex=True, sharey='row')
    for col, (risk, risk_label) in enumerate(RISKS):
        arena_rows = rows_of_risk(arena, risk)
        moar_rows = rows_of_risk(moar, risk)
        for row, (metric, metric_label) in enumerate(metrics_of_rows):
            ax = axes[row, col]
            for label, color, kind, hatch, data in bands:
                values = np.array([d[metric] for d in (rows_of_risk(data, risk) if kind == 'risk' else data)])
                if len(values) == 0:
                    continue
                if kind == 'line' or np.ptp(values) == 0.0:
                    ax.axhline(values.mean(), color=color, linewidth=0.9, zorder=2)
                else:
                    ax.axhspan(values.min(), values.max(), facecolor=color, alpha=BAND_ALPHA, linewidth=0, zorder=1)
                    ax.axhspan(values.min(), values.max(), facecolor='none', edgecolor=color, hatch=hatch,
                               linewidth=0.4, zorder=1)
                band_limits[row].extend([values.min(), values.max()])
            for label, color, style, best in optimal:
                ax.axhline(best[metric], color=color, linestyle=style, linewidth=0.9, zorder=3)
            if moar_rows:
                x = np.array([r[risk] for r in moar_rows]) + X_JITTER
                ax.scatter(x, [r[metric] for r in moar_rows], zorder=4, **MOAR_STYLE)
            if args.sweep_fronts:
                levels, q = quartiles(arena_rows, risk, metric)
                if len(levels) > 25:  # Fine risk step: interquartile band and median line
                    ax.fill_between(levels, q[:, 0], q[:, 2], color=ARENA_STYLE['color'], alpha=0.18, linewidth=0,
                                    zorder=5)
                    ax.plot(levels, q[:, 1], color=ARENA_STYLE['color'], linewidth=0.9, zorder=5)
                else:
                    ax.vlines(levels - X_JITTER, q[:, 0], q[:, 2], color=ARENA_STYLE['color'], linewidth=0.7,
                              zorder=5)
                    ax.scatter(levels - X_JITTER, q[:, 1], zorder=5, **ARENA_STYLE)
            else:
                x = np.array([r[risk] for r in arena_rows]) - X_JITTER
                ax.scatter(x, [r[metric] for r in arena_rows], zorder=5, **ARENA_STYLE)

            ax.grid(True, linewidth=0.4, alpha=0.4)
            ax.set_xlim(-0.06, 1.06)
            ax.set_xticks([0.0, 0.5, 1.0])
            if col == 0:
                ax.set_ylabel(metric_label)
            if row == len(metrics_of_rows) - 1:
                ax.set_xlabel(risk_label)

    for row in range(len(metrics_of_rows)):
        core = np.array(core_values[row])
        margin = Y_MARGIN * np.ptp(core)
        low = min(core.min() - margin, min(band_limits[row], default=np.inf))
        high = core.max() + margin
        axes[row, 0].set_ylim(low - 0.02 * (high - low), high)

    # Legend above the panels: optimal trajectories, adaptive planners, then the bands
    handles = [Line2D([], [], color=c, linestyle=s, linewidth=0.9, label=l) for l, c, s, _ in optimal]
    if args.sweep_fronts and args.every_level:
        handles += [Patch(facecolor=(0.04, 0.04, 0.04, 0.18), edgecolor='#0b0b0b', linewidth=0.9,
                          label='ARENA (median line, IQR band)')]
    else:
        handles += [Line2D([], [], linestyle='none' if not args.sweep_fronts else '-', linewidth=0.7, marker='o',
                       markersize=3, color='#0b0b0b', label='ARENA (median, IQR)' if args.sweep_fronts else
                       'Reference front' if args.reference_front is not None else 'ARENA')]
    if moar:
        handles += [Line2D([], [], linestyle='none', marker='s', markersize=3.5, markerfacecolor='none',
                           markeredgecolor='#52514e', markeredgewidth=0.6, label='MOAR-3D')]
    for label, color, kind, hatch, _ in bands:
        if kind == 'line':
            handles.append(Line2D([], [], color=color, linewidth=0.9, label=label))
        else:
            handles.append(Patch(facecolor=(*plt.matplotlib.colors.to_rgb(color), BAND_ALPHA), edgecolor=color,
                                 hatch=hatch, linewidth=0.5, label=label))
    legend = fig.legend(handles=handles, loc='upper center', ncol=3, frameon=False, fontsize=6.5, handlelength=1.8,
                        columnspacing=0.8, handletextpad=0.4, labelspacing=0.3, bbox_to_anchor=(0.5, 1.0))

    # Panels right below the legend, whatever its number of rows
    fig.canvas.draw()
    legend_bottom = legend.get_window_extent().transformed(fig.transFigure.inverted()).y0
    fig.tight_layout(pad=0.3, h_pad=0.4, w_pad=0.3, rect=(0, 0, 1, legend_bottom - 0.01))
    for extension in ['pdf', 'png']:
        fig.savefig('{}.{}'.format(output, extension), dpi=args.dpi, bbox_inches='tight', pad_inches=0.02)
    print('Figure written to {0}.pdf and {0}.png'.format(output))


if __name__ == '__main__':
    main()
