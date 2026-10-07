#!/bin/bash
# A*: MOAR-3D (weighted A*) with the weights 1/0/0, the shortest path on the grid. The search is deterministic, the
# runs are repeated for the planning time only.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests moar_3d "$BENCHMARK_CONFIGS_DIR/moar_3d_params.yaml" astar 20 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
