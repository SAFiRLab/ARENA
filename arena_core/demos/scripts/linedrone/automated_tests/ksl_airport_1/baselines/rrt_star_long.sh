#!/bin/bash
# RRT* (path length) with a planning time of 5 s and the RRT range of ARENA (5 m). 50 feasible runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests rrt_star "$BENCHMARK_CONFIGS_DIR/rrt_star_long_params.yaml" rrt_star_long 50 \
    fixed_rrt_range:=5.0 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
