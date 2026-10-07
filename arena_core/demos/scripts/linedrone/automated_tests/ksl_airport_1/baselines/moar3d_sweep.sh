#!/bin/bash
# MOAR-3D in weight sweep mode: one A* search per weight set of the simplex (step 0.1, 66 sets) in every planning, the
# solutions are reported like a Pareto front (scalarized multi-objective planning). The chosen trajectory uses the
# voting weights 1/0/0 like ARENA. Deterministic, the runs are repeated for the planning time only.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests moar_3d "$BENCHMARK_CONFIGS_DIR/moar_3d_sweep_params.yaml" moar3d_sweep 20 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
