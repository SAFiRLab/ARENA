#!/bin/bash
# RRT* (path length) with a planning time of about the mean planning time of ARENA with its default hyperparameters on
# the map (0.75 s on map_test_3, BASELINE_RRT_STAR_SHORT_PARAMS for other maps), and the RRT range of ARENA (5 m).
# 50 feasible runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests rrt_star "${BASELINE_RRT_STAR_SHORT_PARAMS:-$BENCHMARK_CONFIGS_DIR/rrt_star_params.yaml}" rrt_star_short 50 \
    fixed_rrt_range:=5.0 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
