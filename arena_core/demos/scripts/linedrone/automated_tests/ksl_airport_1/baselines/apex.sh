#!/bin/bash
# A*pex (approximate multi-objective A*, Zhang et al., ICAPS 2022) on the graph of MOAR-3D: every planning returns an
# epsilon-approximate Pareto front (apex.epsilon) of the time, safety and energy terms of the MOAR cost, reported like a
# front. The chosen trajectory uses the voting weights 1/0/0 like the other baselines. Deterministic: 3 runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests apex "$BENCHMARK_CONFIGS_DIR/apex_params.yaml" apex 3 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
