#!/bin/bash
# ARENA with its default hyperparameters (N_gen = 1000, N_pop = 40, delta_RRT = 5 m, N_nurbs = 50), reference row of the
# comparison with the benchmark planners. 50 feasible runs, cost weights 1/0/0 (time) like the hyperparameter sweeps.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_planning_tests arena "" arena_default 50 \
    fixed_nb_of_generations:=1000 \
    fixed_population_size:=40 \
    fixed_rrt_range:=5.0 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
