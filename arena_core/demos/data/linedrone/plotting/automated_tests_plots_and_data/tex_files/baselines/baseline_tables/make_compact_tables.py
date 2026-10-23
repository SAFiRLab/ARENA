"""Compact baseline tables for the ARENA paper (single-column and full-width layouts).

Each metric cell is either
  * a list of per-run values  -> summarised as median [Q1, Q3] (or [min, max] if n < MIN_N_IQR), or
  * a (mean, std) tuple        -> aggregated values, shown as mean (std in small font), or
  * a scalar                   -> deterministic planner / fixed budget, no spread.
Bold = best among dynamically feasible planners (success > 0), decided on the displayed (rounded) values.
"""
import numpy as np

MIN_N_IQR = 5          # below this, an IQR is meaningless: report [min, max]
SKEW_RATIO = 0.5       # cells with std > 0.5*mean are reported by report_skew() (Gustavii rule of thumb)
ACROSS_WEIGHTS = {("MOAR-3D~\\cite{petit2024} sweep", "time"), ("MOAR-3D kino.\\ sweep", "time")}  # spread over 66 weight sets, not runs

# (key, header, unit, better, formatter-kind)
COLS = [("succ", "SR", "(\\%)", "max", "int"),
        ("time", "Pl.\\,Time", "(s)", "min", "sig"),
        ("dur", "Dur.", "(s)", "min", "d0"),
        ("energy", "Energy", "(kJ)", "min", "d0"),
        ("minclr", "$d_\\text{min}$", "(m)", "max", "d1"),
        ("meanclr", "$d_\\text{mean}$", "(m)", "max", "d1"),
        ("hv", "HV", "", "max", "d2"),
        ("igd", "IGD$^+$", "", "min", "d2")]

ENVS = {"seed42": "Cluttered environment", "cl3": "Powerlines environment"}

# Planner order: dynamically feasible first, then the relaxed (no dynamics) references.
FEASIBLE = ["ARENA (ours)", "MOAR-3D kino.\\ sweep", "Fast-Planner~\\cite{zhou2019}", "Kino.\\ RRT$^*$"]
RELAXED = ["A$^*$", "RRT$^*$", "MOAR-3D~\\cite{petit2024} sweep", "Ahmed and Deb~\\cite{ahmed2011}", "A$^*$pex~\\cite{zhang2022}", "PO-RRT$^*$~\\cite{batonisashvili2026}"]

import json, os
DATA = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "table_data.json")))
DATA["seed42"] = DATA.pop("s42")


def decimals(kind, v):
    if kind == "int": return 0
    if kind.startswith("d"): return int(kind[1])
    a = abs(v)  # "sig": ~2-3 significant figures for values spanning orders of magnitude
    return 3 if a < 0.1 else 2 if a < 10 else 1 if a < 100 else 0


MEDIAN_IQR = {("MOAR-3D~\\cite{petit2024} sweep", "time"), ("MOAR-3D kino.\\ sweep", "time")}   # skewed: spread over the 66 weight sets


def pm(m, s, kind):
    """mean +- std, std in small font; a std below the displayed precision is omitted"""
    d = decimals(kind, m)
    if round(s, d) == 0: return m, "", False
    return m, f"$\\pm${s:.{d}f}".replace("$0.", "$."), False


def summarise(cell, kind, median=False):
    """-> (central value, spread string, flag). Cells are tagged by compute_metrics.py."""
    if cell is None: return None, "", False
    if not isinstance(cell, dict): return cell, "", False                 # deterministic: identical runs
    if "budget" in cell: return cell["budget"], "", False                 # fixed time budget
    if "runs" in cell:                                                     # per-run (or per-search) values
        x = np.asarray(cell["runs"], float)
        if not median: return pm(float(x.mean()), float(x.std(ddof=1)), kind)
        m = float(np.median(x)); d = decimals(kind, m)
        lo, hi = np.percentile(x, [25, 75]) if len(x) >= MIN_N_IQR else (x.min(), x.max())
        iqr = hi - lo; di = d
        if iqr > 0 and round(iqr, di) == 0: di = int(-np.floor(np.log10(iqr)))   # never print (0.00)
        txt = f"{iqr:.{di}f}"; txt = txt[1:] if txt.startswith("0.") else txt
        return m, f"({txt})", False
    return pm(cell["mean"], cell["std"], kind)                             # published aggregate (mean, std)


def fmt(v, kind, d=None):
    return f"{v:.{decimals(kind, v) if d is None else d}f}"


def build_cells(env):
    """Formatted cells per planner for one environment, bold on best feasible (rounded) values."""
    rows = FEASIBLE + RELAXED
    summ = {p: {k: summarise(DATA[env][p][k], kind, (p, k) in MEDIAN_IQR) for k, _, _, _, kind in COLS} for p in rows}
    best = {}
    for k, _, _, better, kind in COLS:
        vals = [float(fmt(summ[p][k][0], kind)) for p in FEASIBLE
                if summ[p][k][0] is not None and DATA[env][p]["succ"] > 0]
        best[k] = (max if better == "max" else min)(vals)
    out = {}
    for p in rows:
        relaxed = p in RELAXED
        cells = []
        for k, _, _, _, kind in COLS:
            v, spread, skew = summ[p][k]
            if v is None: cells.append("--"); continue
            d = len(spread.split(".")[1]) if spread.startswith("$") and "." in spread else None
            s = fmt(v, kind, d)
            if not relaxed and float(fmt(v, kind)) == best[k]: s = f"\\textbf{{{s}}}"
            if (p, k) in MEDIAN_IQR: s += "$^\\dagger$"
            if spread: s += f"\\sd{{{spread}}}"
            cells.append(f"\\rx{{{s}}}" if relaxed else s)
        out[p] = (f"\\rx{{{p}}}" if relaxed else p, cells)
    return out


PREAMBLE = r"""\documentclass[journal]{IEEEtran}
\usepackage{booktabs}
\usepackage{amsmath}
\usepackage[table]{xcolor}
% --- goes in the preamble of the paper ---
\newcommand{\sd}[1]{\,{\fontsize{5}{6}\selectfont #1}}    % spread in small font, same line
\newcommand{\rx}[1]{\textcolor{black!50}{#1}}            % relaxed-problem reference (no dynamics)
\begin{document}
"""

NOTE = (r"SR: success rate. Pl.\,Time: planning time. Dur.: duration. "
        r"$d_\text{min}$, $d_\text{mean}$: minimum and mean clearance. HV: hypervolume. "
        r"IGD$^+$: inverted generational distance plus. \rx{Gray: trajectories violate constraints.} "
        r"$^\dagger$median (IQR) shown instead of mean $\pm$ std because of skewed distribution.")


GROUPS = [("Best of the front", "dur", "meanclr")]   # spanner headers over metric columns


def group_row(n_blocks):
    """Spanner row and its \\cmidrule's, repeated for each environment block (planner column first)."""
    keys = [c[0] for c in COLS]; cells, rules = [""], []
    for b in range(n_blocks):
        off = 2 + b * len(COLS); i = 0
        while i < len(keys):
            g = next((g for g in GROUPS if g[1] == keys[i]), None)
            if g:
                w = keys.index(g[2]) - i + 1
                cells.append(f"\\multicolumn{{{w}}}{{c}}{{{g[0]}}}")
                rules.append(f"\\cmidrule(lr){{{off + i}-{off + i + w - 1}}}"); i += w
            else:
                cells.append(""); i += 1
    return " & ".join(cells) + r" \\", " ".join(rules)


def header_cells():
    return " & ".join(h for _, h, _, _, _ in COLS), " & ".join(u for _, _, u, _, _ in COLS)


# names too wide for one column: first line aligned with the data, continuation indented below
WRAP_SINGLE = {"MOAR-3D kino.\\ sweep": r"MOAR-3D kino.\\ \hspace{0.8em}sweep",
               "MOAR-3D~\\cite{petit2024} sweep": r"MOAR-3D~\cite{petit2024}\\ \hspace{0.8em}sweep"}


def caption():
    """Caption with the number of runs of the non-deterministic planners (their best duration varies between runs)."""
    runs = sorted({DATA[env][p]["n_runs"] for env in ENVS for p in FEASIBLE + RELAXED
                   if isinstance(DATA[env][p]["dur"], dict) and "runs" in DATA[env][p]["dur"]})
    n = str(runs[0]) if len(runs) == 1 else f"{runs[0]}--{runs[-1]}"
    return f"\\caption{{Comparison with baselines ({n} runs per non-deterministic planner)}}"


def single_column():
    h, u = header_cells(); n = len(COLS) + 1
    L = [r"\begin{table}[t]", r"\centering",
         caption(),
         r"\label{tab:baselines}", r"\fontsize{6.5}{7.8}\selectfont", r"\setlength{\tabcolsep}{0.9pt}",
         r"\begin{tabular}{@{}l@{\hspace{5.5pt}}" + "c" * len(COLS) + "@{}}", r"\toprule", *group_row(1),
         f" & {h} \\\\", f"Planner & {u} \\\\"]
    for env, title in ENVS.items():
        cells = build_cells(env)
        L += [r"\midrule", f"\\multicolumn{{{n}}}{{@{{}}l}}{{\\textit{{{title}}}}} \\\\[1pt]"]
        for p in FEASIBLE + RELAXED:
            if p == RELAXED[0]: L.append(f"\\cmidrule(lr){{1-{n}}}")
            name, cs = cells[p]
            if p in WRAP_SINGLE:
                name = r"\begin{tabular}[t]{@{}l@{}}" + WRAP_SINGLE[p] + r"\end{tabular}"
                if p in RELAXED: name = f"\\rx{{{name}}}"   # keep the gray of relaxed rows
            L.append(f"{name} & " + " & ".join(cs) + r" \\")
    L += [r"\bottomrule", r"\end{tabular}", r"\\[2pt]",
          r"\parbox{\columnwidth}{\fontsize{6}{7}\selectfont " + NOTE + "}", r"\end{table}"]
    return "\n".join(L)


def full_width():
    k = len(COLS); h, u = header_cells()
    L = [r"\begin{table*}[t]", r"\centering",
         caption(),
         r"\label{tab:baselines}", r"\scriptsize", r"\setlength{\tabcolsep}{1.2pt}",
         r"\begin{tabular*}{\textwidth}{@{\extracolsep{\fill}}l@{\hspace{10pt}}" + "c" * (2 * k) + "@{}}", r"\toprule",
         " & " + " & ".join(f"\\multicolumn{{{k}}}{{c}}{{{t}}}" for t in ENVS.values()) + r" \\",
         f"\\cmidrule(lr){{2-{k + 1}}} \\cmidrule(lr){{{k + 2}-{2 * k + 1}}}", *group_row(2),
         f" & {h} & {h} \\\\", f"Planner & {u} & {u} \\\\", r"\midrule"]
    c = {e: build_cells(e) for e in ENVS}
    for p in FEASIBLE + RELAXED:
        if p == RELAXED[0]: L.append(f"\\cmidrule(lr){{1-{2 * k + 1}}}")
        e1, e2 = list(ENVS)
        L.append(f"{c[e1][p][0]} & " + " & ".join(c[e1][p][1] + c[e2][p][1]) + r" \\")
    L += [r"\bottomrule", r"\end{tabular*}", r"\\[2pt]",
          r"\parbox{\textwidth}{\fontsize{6}{7}\selectfont " + NOTE + "}", r"\end{table*}"]
    return "\n".join(L)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Compact baseline tables from the JSON of compute_metrics.py")
    parser.add_argument("--data", default="table_data.json", help="JSON of compute_metrics.py (--output)")
    parser.add_argument("--suffix", default="", help="Added to the table file names (variants), e.g. _g300_rrt10")
    args = parser.parse_args()
    DATA.clear()
    DATA.update(json.load(open(args.data)))
    DATA["seed42"] = DATA.pop("s42")
    for name, body in [("table_baselines_single_column" + args.suffix, single_column()),
                       ("table_baselines_full_width" + args.suffix, full_width())]:
        with open(f"{name}.tex", "w") as f:
            f.write(PREAMBLE + "% ---- PAPER TEXT (start) ----\n" + body +
                    "\n% ---- PAPER TEXT (end) ----\n\\end{document}\n")
