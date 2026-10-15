#!/bin/bash
# MOAR-3D in weight sweep mode: one MOAR search per coefficient set (k_S, k_T, k_E) of the simplex (step 0.1, 66 sets) in
# every planning, the solutions are reported like a Pareto front. The chosen trajectory uses the voting weights 1/0/0 like
# ARENA. About 2.5 min per planning (the sets with k_T = 0 search almost the whole graph), deterministic: 3 runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests moar_3d "$BENCHMARK_CONFIGS_DIR/moar_3d_sweep_params.yaml" moar3d_sweep 3 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
