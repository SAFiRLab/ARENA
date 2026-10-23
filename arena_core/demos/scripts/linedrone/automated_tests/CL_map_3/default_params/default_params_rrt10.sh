#!/bin/bash
# ARENA with its default hyperparameters (linedrone_problem_params.yaml: 1000 generations, population 40, 50 samples)
# except the RRT range of the initialization, 10 m, on the planning problem of the reference front of this map. Exactly
# num_iter runs (hyperparameter_max_nb_of_runs), every run writes its front in report_3d_default_params_<date>_pareto_front.csv
# and every solution of its report with its clearance in report_3d_default_params_<date>_solutions.csv (fronts of the
# risk sensitivity figure with default parameters, plot_risk_sensitivity.py).
#
# The reports are written in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d).

source "$(dirname "$(readlink -f "$0")")/../../terminal_utils.sh"

# Same planning problem as the hyperparameter sweeps (scripts/.../hyperparameters_variation/*.sh)
map_name="map_test_3"

# Default hyperparameters, RRT range of 10 m
rrt_range=10.0
nb_of_generations=1000
population_size=40
nurbs_sample_size=50
num_iter=50

# Every solution of the reports exported with its clearance (average and minimum distance to the inflated octomap on the
# samples of the final evaluation) next to the reports, to plot the clearance of the fronts (analysis only, not
# part of the planning time)
solutions_file="/home/dev_ws/data_output/navigation_3d/report_3d_default_params_$(date +%Y-%m-%d_%H-%M-%S)_solutions.csv"
start_planner="$(dirname "$(readlink -f "$0")")/../../start_planner_node.sh"

trap 'trap - INT TERM; stop_testbench_nodes; exit 1' INT TERM

if [ "$TESTBENCH_TERMINAL" = "tmux" ]; then
    echo "The nodes run in tmux session '$TESTBENCH_TMUX_SESSION', watch them from another terminal with: tmux attach -t $TESTBENCH_TMUX_SESSION"
fi

echo "===== Default hyperparameters: ${num_iter} run(s), ${nb_of_generations} generations, population ${population_size}, RRT range ${rrt_range} m ====="

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
    world_name:=default_params \
    planner_start_command:='${start_planner} arena ${solutions_file}' \
    is_hyperparameter_variation_tests:=true \
    is_step_variation_tests:=false \
    hyperparameter_name:=rrt_range \
    hyperparameter_min:=${rrt_range} \
    hyperparameter_max:=${rrt_range} \
    hyperparameter_steps:=1.0 \
    hyperparameter_nb_of_iter:=${num_iter} \
    hyperparameter_max_nb_of_runs:=${num_iter} \
    fixed_nb_of_generations:=${nb_of_generations} \
    fixed_population_size:=${population_size} \
    fixed_nurbs_sample_size:=${nurbs_sample_size}"

until [ "$(grep -c "$end_message" /tmp/testbench.log 2> /dev/null)" -gt "$nb_of_ends" ]; do
    sleep 10
done

echo "===== Default hyperparameters done ====="
stop_testbench_nodes

echo "The reports are in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d)"
