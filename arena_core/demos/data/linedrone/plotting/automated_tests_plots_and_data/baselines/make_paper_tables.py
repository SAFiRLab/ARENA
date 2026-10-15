"""LaTeX tables of the paper comparing ARENA with the baselines, one per environment (RA-L, IEEEtran two-column).

Reads the CSV written by hyperparameter_variations/compare_baselines.py for every environment
(baselines/<map>/native/baselines_table_native.csv) and writes a standalone IEEEtran document per environment in
tex_files/baselines/baseline_tables/ (the table* between the two "PAPER TEXT" markers goes in the paper). Every cell is
the mean over the runs with the standard deviation below it (omitted when it is zero); the best value of every column is
in bold (no best for the front size).

Variants (e.g. ARENA with another setting): --variant <name> reads baselines_table_native_<name>.csv and writes
table_baselines_<map>_<name>.tex (labels suffixed with _<name>), --note adds a sentence to the notes under the table.

Usage:
    python3 make_paper_tables.py [--output-dir <folder>] [--variant <name> --note <text>]
"""

import argparse
import csv
import math
import os

HERE = os.path.dirname(os.path.realpath(__file__))
TEX_DIR = os.path.join(HERE, '..', 'tex_files', 'baselines', 'baseline_tables')

ENVIRONMENTS = [
    # (map folder, file name, environment name in the caption, label)
    ('CL_map_3', 'table_baselines_cl_map_3', 'the CL\\_map\\_3 environment', 'tab:baselines_cl_map_3'),
    ('seed_42', 'table_baselines_seed_42', 'the cluttered environment (seed 42)', 'tab:baselines_seed_42'),
]

# (method in the CSV, name in the table), grouped: (group name, methods)
GROUPS = [
    ('', [('ARENA', 'ARENA (ours)')]),
    ('Single-objective', [('A*', 'A$^*$'), ('RRT* (0.75 s)', 'RRT$^*$'), ('RRT* (0.9 s)', 'RRT$^*$')]),
    ('Weighted sum', [('MOAR-3D', 'MOAR-3D'), ('MOAR-3D (weight sweep)', 'MOAR-3D sweep')]),
    ('Pareto front', [('Spline NSGA-II', 'Spline NSGA-II'), ('A*pex', 'A$^*$pex'), ('PO-RRT*', 'PO-RRT$^*$')]),
    ('Dynamics-aware', [('Kinodynamic A*', 'Kino. A$^*$'), ('Kinodynamic RRT* (5 s)', 'Kino. RRT$^*$ (5\\,s)'),
                        ('Kinodynamic RRT* (60 s)', 'Kino. RRT$^*$ (60\\,s)')]),
]

# (CSV header, column header, decimals or None (adaptive), best: 'max' | 'min' | None)
COLUMNS = [
    ('Completion (%)', 'Compl.', 0, 'max'),
    ('Feasible (%)', 'Feas.', 0, 'max'),
    ('Phys. feasible (%)', 'Phys.', 0, 'max'),
    ('Planning time (s)', 'Time', None, 'min'),
    ('Best duration (s)', 'Dur.', 1, 'min'),
    ('Best energy (kJ)', 'Energy', 1, 'min'),
    ('Safest min clearance (m)', 'Min.\\ clr.', 2, 'max'),
    ('Safest mean clearance (m)', 'Mean clr.', 2, 'max'),
    ('Front size', 'Size', 1, None),
    ('HV ratio', 'HV', 3, 'max'),
    ('IGD+', 'IGD$^+$', 3, 'min'),
]

# Grouped headers: (title, number of columns)
HEADER_GROUPS = [
    ('Success rate (\\%)', 3),
    ('Planning', 1),
    ('Best of the front', 4),
    ('Front quality', 3),
]
UNITS = ['', '', '', '(s)', '(s)', '(kJ)', '(m)', '(m)', '', '', '']


# Planners returning a single trajectory: no front quality (front size, HV, IGD+)
NO_FRONT_QUALITY = {'A*', 'RRT* (0.75 s)', 'RRT* (0.9 s)', 'MOAR-3D'}
FRONT_QUALITY_HEADERS = {'Front size', 'HV ratio', 'IGD+'}

# Folders of the runs exporting their solutions (optimization.export_solutions_file), for the clearance of the safest
# trajectory of the front. The deterministic planners have one export run (<folder>_export), every run gives the same
# trajectories.
SOLUTION_FOLDERS = {
    'ARENA': ['arena_both2pct'], 'A*': ['astar_export'], 'RRT* (0.75 s)': ['rrt_star'], 'RRT* (0.9 s)': ['rrt_star'],
    'MOAR-3D': ['moar3d_export'], 'MOAR-3D (weight sweep)': ['moar3d_sweep_export'], 'Spline NSGA-II': ['spline_nsga2'],
    'A*pex': ['apex_export'], 'PO-RRT*': ['po_rrt_star'], 'Kinodynamic A*': ['kino_astar_export'],
    'Kinodynamic RRT* (60 s)': ['kino_rrt_star'],
}


def safest_clearance(folder):
    """(mean, std) of the min and of the mean clearance of the safest trajectory (lowest safety cost) of every planning,
    from the solutions.csv of a run folder, None without it."""
    path = os.path.join(folder, 'solutions.csv')
    if not os.path.exists(path):
        return None
    # One block of consecutive rows per planning (the planning index restarts when the testbench restarts the planner)
    best, block, previous = {}, -1, None
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row['Planning'] != previous:
                block += 1
                previous = row['Planning']
            safety = float(row['Security cost'])
            if block not in best or safety < best[block][0]:
                best[block] = (safety, float(row['Min Closest Obstacle Distance']),
                               float(row['Average Closest Obstacle Distance']))
    if not best:
        return None

    def stats(values):
        mean = sum(values) / len(values)
        std = math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1)) if len(values) > 1 else 0.0
        return mean, std
    return stats([b[1] for b in best.values()]), stats([b[2] for b in best.values()])


def read_csv(path):
    with open(path, newline='') as f:
        return {row['Method']: row for row in csv.DictReader(f)}


def value(row, header):
    """(mean, std) of a metric, NaN when the planner has no feasible run."""
    def number(text):
        try:
            return float(text)
        except (TypeError, ValueError):
            return math.nan
    return number(row.get(header + ' mean')), number(row.get(header + ' std'))


def decimals_for(mean, decimals):
    if decimals is not None:
        return decimals
    magnitude = abs(mean)
    return 3 if magnitude < 1.0 else 2 if magnitude < 10.0 else 1


def cell(mean, std, decimals, bold):
    if math.isnan(mean):
        return '--'
    d = decimals_for(mean, decimals)
    text = '{:.{d}f}'.format(mean, d=d)
    if bold:
        text = '\\textbf{' + text + '}'
    if not math.isnan(std) and round(std, d) > 0.0:
        return '\\ms{' + text + '}{' + '{:.{d}f}'.format(std, d=d) + '}'
    return text


def sweep_search_time(map_folder, data_dir='native'):
    """(mean, std) of the time of one weighted search of the MOAR-3D sweep (66 searches per planning), from
    native/moar3d_sweep/search_times.csv (extracted from the planner logs of the reported runs)."""
    path = os.path.join(HERE, map_folder, data_dir, 'moar3d_sweep', 'search_times.csv')
    with open(path, newline='') as f:
        times = [float(row['Search time (s)']) for row in csv.DictReader(f)]
    mean = sum(times) / len(times)
    std = math.sqrt(sum((t - mean) ** 2 for t in times) / (len(times) - 1))
    return mean, std


def table(rows, environment, label, nb_of_runs, note=''):
    methods = [(group, csv_name, name) for group, members in GROUPS for csv_name, name in members if csv_name in rows]

    # Best value of every column (rounded as displayed)
    best = []
    for header, _, decimals, direction in COLUMNS:
        values = []
        for _, csv_name, _ in methods:
            mean, _ = value(rows[csv_name], header)
            if not math.isnan(mean):
                values.append(round(mean, decimals_for(mean, decimals)))
        best.append(None if direction is None or not values else (max(values) if direction == 'max' else min(values)))

    lines = []
    lines.append('\\begin{table*}[t]')
    lines.append('\\centering')
    lines.append('\\caption{Comparison of ARENA with the baselines in ' + environment + ' (' + nb_of_runs +
                 ' runs per planner, mean $\\pm$ standard deviation)}')
    lines.append('\\label{' + label + '}')
    lines.append('\\scriptsize')
    lines.append('\\setlength{\\tabcolsep}{2.2pt}')
    lines.append('\\begin{tabular}{l' + 'c' * len(COLUMNS) + '}')
    lines.append('\\toprule')
    header_cells, rules, start = [], [], 2
    for title, span in HEADER_GROUPS:
        header_cells.append('\\multicolumn{' + str(span) + '}{c}{' + title + '}')
        rules.append('\\cmidrule(lr){' + str(start) + '-' + str(start + span - 1) + '}')
        start += span
    lines.append(' & ' + ' & '.join(header_cells) + ' \\\\')
    lines.append(' '.join(rules))
    lines.append('Planner & ' + ' & '.join(c[1] for c in COLUMNS) + ' \\\\')
    lines.append(' & ' + ' & '.join(UNITS) + ' \\\\')
    lines.append('\\midrule')

    previous_group = None
    for group, csv_name, name in methods:
        if group != previous_group and previous_group is not None:
            lines.append('\\midrule')
        if group != previous_group and group:
            lines.append('\\multicolumn{' + str(len(COLUMNS) + 1) + '}{l}{\\textit{' + group + '}} \\\\')
        previous_group = group
        cells = []
        for (header, _, decimals, _), best_value in zip(COLUMNS, best):
            mean, std = value(rows[csv_name], header)
            shown = None if math.isnan(mean) else round(mean, decimals_for(mean, decimals))
            cells.append(cell(mean, std, decimals, best_value is not None and shown == best_value))
        lines.append(name + ' & ' + ' & '.join(cells) + ' \\\\')
    lines.append('\\bottomrule')
    lines.append('\\end{tabular}')
    lines.append('')
    lines.append('\\vspace{2pt}')
    lines.append('\\parbox{0.92\\textwidth}{\\scriptsize Success rates: share of the runs returning a trajectory that '
                 'reaches the goal (Compl.), that also respects the constraints of the method and is collision-free (Feas.), '
                 'and that respects all the physical limits of the robot, the acceleration included (Phys.). Every trajectory '
                 'is evaluated as planned with the cost models of ARENA; the planners without dynamics change their velocity '
                 'instantaneously. Planning time of the MOAR-3D sweep: time of one of its 66 weighted searches. Best of the '
                 'front: best duration and energy among the solutions of a run, and minimum and mean clearance (distance to '
                 'the inflated octomap) of its safest solution (lowest safety cost). Front quality: hypervolume ratio and IGD$^+$ against the non-dominated union '
                 'of all the fronts and of the ARENA reference runs. Best values in bold.' + (' ' + note if note else '') + '}')
    lines.append('\\end{table*}')
    return '\n'.join(lines)


PREAMBLE = r'''\documentclass[journal]{IEEEtran}
\usepackage{booktabs}
\usepackage{amsmath}

% Mean with the standard deviation below it (goes in the preamble of the paper)
\newcommand{\ms}[2]{\begin{tabular}[c]{@{}c@{}}#1\\[-1.5pt]{\tiny$\pm$#2}\end{tabular}}

% Standalone preview of a table of the paper. Only the table between the two "PAPER TEXT" markers goes in the paper.
\begin{document}

% ---------------------------------------- PAPER TEXT (start) ----------------------------------------
'''

POSTAMBLE = r'''
% ---------------------------------------- PAPER TEXT (end) ----------------------------------------

\end{document}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output-dir', default=TEX_DIR)
    parser.add_argument('--variant', default='', help='Suffix of the CSV and of the tex files and labels')
    parser.add_argument('--note', default='', help='Sentence added to the notes under the table')
    parser.add_argument('--data-dir', default='native', help='Folder of the runs in baselines/<map>/ (e.g. native_res)')
    parser.add_argument('--arena-folder', default='', help='Folder of the ARENA runs exporting their solutions (variants)')
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    for map_folder, file_name, environment, label in ENVIRONMENTS:
        suffix = '_' + args.variant if args.variant else ''
        path = os.path.join(HERE, map_folder, args.data_dir, 'baselines_table_native' + suffix + '.csv')
        rows = read_csv(path)
        # The planning time of the MOAR-3D sweep is the time of one of its weighted searches
        sweep = rows.get('MOAR-3D (weight sweep)')
        if sweep is not None:
            sweep['Planning time (s) mean'], sweep['Planning time (s) std'] = sweep_search_time(map_folder, args.data_dir)
        # Clearance of the safest trajectory of the front, from the exported solutions
        for method, row in rows.items():
            folders = [args.arena_folder] if method == 'ARENA' and args.arena_folder else SOLUTION_FOLDERS.get(method, [])
            for folder in folders:
                result = safest_clearance(os.path.join(HERE, map_folder, args.data_dir, folder))
                if result:
                    (row['Safest min clearance (m) mean'], row['Safest min clearance (m) std']), \
                        (row['Safest mean clearance (m) mean'], row['Safest mean clearance (m) std']) = result
                    break
        for method in NO_FRONT_QUALITY & set(rows):
            for header in FRONT_QUALITY_HEADERS:
                rows[method][header + ' mean'] = rows[method][header + ' std'] = 'nan'
        nb_of_runs = sorted({int(r['Runs']) for r in rows.values()})
        runs_text = str(nb_of_runs[0]) if len(nb_of_runs) == 1 else '{}--{}'.format(nb_of_runs[0], nb_of_runs[-1])
        output = os.path.join(args.output_dir, file_name + suffix + '.tex')
        with open(output, 'w') as f:
            f.write('% Generated by baselines/make_paper_tables.py from ' + os.path.relpath(path, os.path.join(HERE, '..')) + '\n')
            f.write(PREAMBLE + table(rows, environment, label + suffix, runs_text, args.note) + POSTAMBLE)
        print('Written: {} ({} planners, runs {})'.format(output, len(rows), runs_text))


if __name__ == '__main__':
    main()
