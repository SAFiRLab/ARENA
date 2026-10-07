#!/bin/bash
# Spline NSGA-II of Ahmed and Deb [13, 14] with the same number of generations and population size as ARENA
# (1000 x 40), random initialization, 8 free control points. 50 feasible runs.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests spline_nsga2 "$BENCHMARK_CONFIGS_DIR/spline_nsga2_params.yaml" spline_nsga2 50 \
    fixed_nb_of_generations:=1000 \
    fixed_population_size:=40 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
