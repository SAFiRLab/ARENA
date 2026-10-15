#!/bin/bash
# A*: shortest path on the 3D grid (benchmark "A*" of the MOAR paper), flown at the maximum speed. The search is
# deterministic, the runs are repeated for the planning time only.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests moar_3d "$BENCHMARK_CONFIGS_DIR/astar_params.yaml" astar 20
