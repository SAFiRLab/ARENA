#!/bin/bash
# MOAR-3D at its recommended coefficients k_S,0 = k_T,0 = k_E,0 = 0.33 (MOAR paper, risks null). Deterministic, the runs
# are repeated for the planning time only.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests moar_3d "$BENCHMARK_CONFIGS_DIR/moar_3d_params.yaml" moar3d 20 \
    fixed_cost_time:=0.33 fixed_cost_safety:=0.33 fixed_cost_energy:=0.33
