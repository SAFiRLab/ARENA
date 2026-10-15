#!/bin/bash
# PO-RRT* (Pareto-optimal RRT*, path length and log-survival) with a growth time of 5 s and the RRT range of ARENA (5 m):
# every planning returns the Pareto front of its complete paths, reported like a front. 20 feasible runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests po_rrt_star "$BENCHMARK_CONFIGS_DIR/po_rrt_star_params.yaml" po_rrt_star 20 \
    fixed_rrt_range:=5.0 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
