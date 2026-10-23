"""Raw benchmark data of the baseline tables, read by compute_metrics.py (from load import *).

The runs are the testbench outputs in baselines/<map>/native_res/<folder> (resolution-based final evaluation):
    report_3d_*.csv                one row per trajectory sample of the chosen solution of every planning (Id)
    report_3d_*_pareto_front.csv   the solutions of every planning (Id) with their costs and safety
    status.csv                     completion and constraint flags of every planning (BenchmarkPlannerNode)
    solutions.csv                  the safe solutions of every planning with their clearances (export_solutions_file)
    search_times.csv               time of every weighted search of the MOAR-3D sweeps (66 per planning)

F[(env, planner)] holds the paths of the files of a planner ('rep', 'pf', and 'st', 'sol', 'times' when they exist,
'export_sol': solutions.csv of the export run <folder>_export of the deterministic planners).

Pareto front quality (pareto_quality): hypervolume ratio and IGD+ of every feasible run against the reference front of
the map, the non-dominated union of the ARENA reference runs (REFERENCE_FRONTS) and of the safe solutions of every
planner of FRONT_UNION, HV reference point 1.1 x the worst normalized value (compute_pareto_metrics.evaluate).
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
BASELINES = os.path.join(DATA, 'baselines')
sys.path.insert(0, os.path.join(DATA, 'hyperparameter_variations'))
from compute_pareto_metrics import evaluate, find_reports, load_runs  # noqa: E402

MAPS = {'s42': 'seed_42', 'cl3': 'CL_map_3'}
FOLDERS = {'ARENA': 'arena_g300_rrt10', 'MOARkino': 'moar3d_kino_sweep', 'KinoAstar': 'kino_astar',
           'KinoRRT': 'kino_rrt_star', 'Astar': 'astar', 'RRT': 'rrt_star', 'MOAR': 'moar3d_sweep',
           'NSGA': 'spline_nsga2', 'Apex': 'apex', 'PORRT': 'po_rrt_star'}
# Runs of the ARENA reference fronts (reference_front.sh) of every map
REFERENCE_FRONTS = {'s42': os.path.join(DATA, 'hyperparameter_variations', 'generated_maps', 'seed_42_init2pct_res',
                                        'reference_front'),
                    'cl3': os.path.join(DATA, 'hyperparameter_variations', 'CL_map_3_init2pct_res', 'reference_front')}
# Planners whose safe solutions make the reference front with the reference runs: the planners of the table and the
# kinodynamic RRT* with a 5 s budget (no feasible run on these maps)
FRONT_UNION = list(FOLDERS.values()) + ['kino_rrt_star_5s']


def _files(folder):
    reports = sorted(f for f in glob.glob(os.path.join(folder, 'report_3d_*.csv')) if not f.endswith('_pareto_front.csv'))
    if len(reports) != 1:
        return None
    files = {'rep': reports[0], 'pf': reports[0].replace('.csv', '_pareto_front.csv')}
    for key, name in [('st', 'status.csv'), ('sol', 'solutions.csv'), ('times', 'search_times.csv')]:
        if os.path.isfile(os.path.join(folder, name)):
            files[key] = os.path.join(folder, name)
    if os.path.isfile(os.path.join(folder + '_export', 'solutions.csv')):
        files['export_sol'] = os.path.join(folder + '_export', 'solutions.csv')
    return files


F = {}


def configure(arena_folder='arena_g300_rrt10'):
    """Selects the ARENA runs (folder of native_res, e.g. a variant with other hyperparameters) and indexes the files of
    every planner. The ARENA runs are also part of the reference front of the front quality (FRONT_UNION)."""
    FOLDERS['ARENA'] = arena_folder
    FRONT_UNION[:] = list(FOLDERS.values()) + ['kino_rrt_star_5s']
    F.clear()
    _QUALITY.clear()
    for env, map_folder in MAPS.items():
        for planner, folder in FOLDERS.items():
            files = _files(os.path.join(BASELINES, map_folder, 'native_res', folder))
            if files:
                F[(env, planner)] = files


def report(env, p):
    return pd.read_csv(F[(env, p)]['rep'])


def front(env, p):
    return pd.read_csv(F[(env, p)]['pf'])


def status(env, p):
    return pd.read_csv(F[(env, p)]['st'])


def sol(env, p):
    return pd.read_csv(F[(env, p)]['sol'])


def export_sol(env, p):
    """Safe solutions of the export run of a deterministic planner (every run gives the same trajectories)."""
    return pd.read_csv(F[(env, p)]['export_sol'])


_QUALITY = {}


def pareto_quality(env):
    """{planner: {'hv': [...], 'igd': [...]}}: hypervolume ratio and IGD+ of every feasible run of the planners."""
    if env not in _QUALITY:
        runs = {folder: [] for folder in FRONT_UNION}
        for folder in FRONT_UNION:
            for report_path, pareto_path in find_reports([os.path.join(BASELINES, MAPS[env], 'native_res', folder)]):
                runs[folder] += load_runs(report_path, pareto_path)
        reference_runs = []
        for report_path, pareto_path in find_reports([REFERENCE_FRONTS[env]]):
            reference_runs += load_runs(report_path, pareto_path, all_safe_solutions=True)
        evaluate([r for folder in FRONT_UNION for r in runs[folder]], reference_runs, 1.1, 'worst')
        _QUALITY[env] = {p: {'hv': [r['hypervolume_ratio'] for r in runs[folder] if r['feasible']],
                             'igd': [r['igd_plus'] for r in runs[folder] if r['feasible']]}
                         for p, folder in FOLDERS.items()}
    return _QUALITY[env]


configure()


def plan_time(env, p):
    """Time of every weighted search of a MOAR-3D sweep (s)."""
    return pd.read_csv(F[(env, p)]['times'])['Search time (s)']


def best_of_front(solutions):
    """Per planning: best duration (s) and energy (kJ) among its safe solutions, minimum and mean clearance (m) of its
    safest solution (lowest safety cost). A planning is a block of consecutive rows (the planning index restarts when
    the testbench restarts the planner), as in make_paper_tables.safest_clearance."""
    block = (solutions['Planning'] != solutions['Planning'].shift()).cumsum()
    best = {'dur': [], 'energy': [], 'minclr': [], 'meanclr': []}
    for _, rows in solutions.groupby(block, sort=True):
        safest = rows.loc[rows['Security cost'].idxmin()]
        best['dur'].append(rows['Time cost'].min())
        best['energy'].append(rows['Energy cost'].min() / 1000.0)
        best['minclr'].append(safest['Min Closest Obstacle Distance'])
        best['meanclr'].append(safest['Average Closest Obstacle Distance'])
    return {k: np.asarray(v, float) for k, v in best.items()}
