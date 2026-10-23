#!/bin/bash
# Comparison of ARENA with the benchmark planners on the generated map with seed 42: the scripts of
# CL_map_3/baselines with the planning problem of ../map_settings.sh (map, start and goal) and an RRT* planning time
# of 0.9 s (mean planning time of ARENA with its default hyperparameters on this map). About 1 h.
# Copy the reports to automated_tests_plots_and_data/baselines/seed_42/native_res/<planner>/ for compute_metrics.py.
source "$(dirname "$(readlink -f "$0")")/../map_settings.sh"
export BASELINE_MAP_NAME="$map_name"
export BASELINE_BT_FILE="$bt_file"
export BASELINE_RRT_STAR_SHORT_PARAMS="/home/dev_ws/src/arena_core/demos/config/linedrone/benchmarks/rrt_star_seed42_params.yaml"

"$(dirname "$(readlink -f "$0")")/../../../CL_map_3/baselines/run_all.sh"
