#!/bin/bash
# Estimated optimal (reference) Pareto front of the 3 objectives (time, safety, energy).
#
# The planner is run with over-designed hyperparameters until num_iter runs have found a safe trajectory. Every run
# writes the whole non-dominated front of its final population (3 objectives) in
# report_3d_reference_<date>_pareto_front.csv, one row per solution with the run Id, and the safety of every solution
# (Safe: no segment of the trajectory crosses an obstacle, checked by the planner on its whole final population). The
# file also has the safe solutions only dominated by unsafe ones (In generated front = 0). The estimated Pareto front
# is the non-dominated set of the union of the safe solutions of all the runs, rebuilt by plot_pareto_front_3d.py (also
# saved as CSV), compute_pareto_metrics.py and plot_computation_analysis.py.
# The runs without safe trajectory are kept in the reports (Feasible = 0). The testbench stops after 3 x num_iter runs
# if num_iter feasible runs can't be reached (testbench_launch.py argument hyperparameter_max_nb_of_runs).
# The cost weights only select the trajectory reported as chosen in the main report (report_3d_reference_<date>.csv),
# they don't change the optimization nor the fronts.
#
# The reports are written in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d).
#
# The planning problem (map, start and goal) must be the same as the one of the hyperparameter sweeps the reference
# front is used for: map_name sets the goal (config/linedrone/testbench_configs/<map_name>.yaml) and must match the
# map loaded by costmap_3D_node (config/linedrone/costmap_3D_params.yaml).
#
# Over-designed hyperparameters: every one of them at or above the top of the hyperparameter sweeps of CL_map_3.
# The population size is the upper bound of the number of solutions of the final front.
# NSGA-II needs a population size divisible by 4, the planner rounds it up (250 -> 252).
#
# Duration: about 45 s per run on a Ryzen 9 7950X (planning time model fitted on the sweeps, +40 % for the 2 m RRT
# range), so about 6-7 h for 500 runs.

source "$(dirname "$(readlink -f "$0")")/../../terminal_utils.sh"

# Same planning problem as the hyperparameter sweeps (scripts/.../hyperparameters_variation/*.sh)
map_name="map_test_3"

# Over-designed hyperparameters
rrt_range=2.0
nb_of_generations=5000
population_size=250
nurbs_sample_size=50
num_iter=500

trap 'trap - INT TERM; stop_testbench_nodes; exit 1' INT TERM

if [ "$TESTBENCH_TERMINAL" = "tmux" ]; then
    echo "The nodes run in tmux session '$TESTBENCH_TMUX_SESSION', watch them from another terminal with: tmux attach -t $TESTBENCH_TMUX_SESSION"
fi

echo "===== Reference front: ${num_iter} run(s), ${nb_of_generations} generations, population ${population_size}, RRT range ${rrt_range} m ====="

# The end of the test is detected in the testbench log, which is appended by every run
end_message="End of the hyperparameters variation tests"
nb_of_ends=$(grep -c "$end_message" /tmp/testbench.log 2> /dev/null)
nb_of_ends=${nb_of_ends:-0}

run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py"

# The testbench starts the planner by itself.
# The RRT range is the varied hyperparameter with a single value, the other hyperparameters are fixed.
sleep 1.0
run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
    map_name:=${map_name} \
    world_name:=reference \
    is_hyperparameter_variation_tests:=true \
    is_step_variation_tests:=false \
    hyperparameter_name:=rrt_range \
    hyperparameter_min:=${rrt_range} \
    hyperparameter_max:=${rrt_range} \
    hyperparameter_steps:=1.0 \
    hyperparameter_nb_of_iter:=${num_iter} \
    fixed_nb_of_generations:=${nb_of_generations} \
    fixed_population_size:=${population_size} \
    fixed_nurbs_sample_size:=${nurbs_sample_size}"

until [ "$(grep -c "$end_message" /tmp/testbench.log 2> /dev/null)" -gt "$nb_of_ends" ]; do
    sleep 10
done

echo "===== Reference front done ====="
stop_testbench_nodes

echo "The reports are in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d)"
