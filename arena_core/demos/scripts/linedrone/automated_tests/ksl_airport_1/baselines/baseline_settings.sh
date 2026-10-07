#!/bin/bash
# Common settings of the comparison of ARENA with the benchmark planners on map_test_3 (CL_map_3 data).
# Source this file, don't execute it.
#
# Every planner solves the planning problem of the hyperparameter sweeps and of the reference front of this map
# (map_test_3, start and goal of config/linedrone/testbench_configs/map_test_3.yaml), with the same sample size (50) so
# that their costs are comparable to the reference front. The benchmark planners are started by the testbench in place
# of linedrone_test_node (start_planner_node.sh <planner> <params file>), with the same ROS interface and the same
# reports, see nodes/testbench/benchmarks.
#
# The reports are written in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d), copy them
# to demos/data/linedrone/plotting/automated_tests_plots_and_data/baselines/CL_map_3/<planner>/ for the analysis
# (compare_baselines.py).

BASELINES_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
AUTOMATED_TESTS_DIR="$BASELINES_DIR/../.."
BENCHMARK_CONFIGS_DIR="/home/dev_ws/src/arena_core/demos/config/linedrone/benchmarks"
source "$AUTOMATED_TESTS_DIR/terminal_utils.sh"

map_name="map_test_3"
# Same sample size as the reference front of CL_map_3 (ksl_airport_1/reference_front/reference_front.sh)
nurbs_sample_size=50

trap 'trap - INT TERM; stop_testbench_nodes; exit 1' INT TERM

# Usage: run_testbench <planner> <params file> <world name> <end message> <testbench launch arguments...>
#   planner: arena (linedrone_test_node), moar_3d, rrt_star or spline_nsga2
#   params file: parameters of the benchmark planner (empty for arena or the default parameters of the planner)
#   end message: message of the testbench log telling the end of the test
run_testbench() {
    local planner="$1"
    local params_file="$2"
    local world_name="$3"
    local end_message="$4"
    shift 4

    local start_command="$AUTOMATED_TESTS_DIR/start_planner_node.sh"
    local kill_pattern="lib/arena_core/[l]inedrone_test_node"
    if [ "$planner" != "arena" ]; then
        start_command="$start_command $planner $params_file"
        kill_pattern="lib/arena_core/[${planner:0:1}]${planner:1}_node"
    fi

    if [ "$TESTBENCH_TERMINAL" = "tmux" ]; then
        echo "The nodes run in tmux session '$TESTBENCH_TMUX_SESSION', watch them from another terminal with: tmux attach -t $TESTBENCH_TMUX_SESSION"
    fi
    echo "===== ${planner} (${world_name}) on ${map_name} ====="

    # The end of the test is detected in the testbench log, which is appended by every run
    local nb_of_ends
    nb_of_ends=$(grep -c "$end_message" /tmp/testbench.log 2> /dev/null)
    nb_of_ends=${nb_of_ends:-0}

    run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py"

    # The testbench starts the planner by itself
    sleep 1.0
    run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
        map_name:=${map_name} \
        world_name:=${world_name} \
        planner_start_command:='${start_command}' \
        planner_kill_command:=\"pkill -INT -f '${kill_pattern}'\" \
        $*"

    until [ "$(grep -c "$end_message" /tmp/testbench.log 2> /dev/null)" -gt "$nb_of_ends" ]; do
        sleep 10
    done

    echo "===== ${planner} (${world_name}) done ====="
    stop_testbench_nodes
    echo "The reports are in the testbench output folder (default: /home/dev_ws/data_output/navigation_3d)"
}

# Usage: run_planning_tests <planner> <params file> <world name> <nb of feasible runs> [testbench launch arguments...]
# Repeated plannings with the planning report and the Pareto fronts (hyperparameters variation tests with the sample
# size as single "varied" hyperparameter). The runs without safe trajectory are kept (Feasible = 0), the test stops
# after 3 x <nb of feasible runs> runs if they can't be reached.
run_planning_tests() {
    local planner="$1"
    local params_file="$2"
    local world_name="$3"
    local nb_of_runs="$4"
    shift 4

    run_testbench "$planner" "$params_file" "$world_name" "End of the hyperparameters variation tests" \
        is_hyperparameter_variation_tests:=true \
        is_step_variation_tests:=false \
        is_risks_variation_tests:=false \
        hyperparameter_name:=nurbs_sample_size \
        hyperparameter_min:=${nurbs_sample_size} \
        hyperparameter_max:=${nurbs_sample_size} \
        hyperparameter_steps:=1.0 \
        hyperparameter_nb_of_iter:=${nb_of_runs} \
        "$@"
}

# Usage: run_risk_sweep <planner> <params file> <world name> <nb of runs per risk level> [testbench launch arguments...]
# Every risk (battery, wind, location) swept from 0 to 1 by 0.1, the others null, the cost weights of the planner given
# by Eq. 11 from the initial weights (0.33, 0.33, 0.33). Writes the risks report (data of Fig. 4).
run_risk_sweep() {
    local planner="$1"
    local params_file="$2"
    local world_name="$3"
    local nb_of_runs="$4"
    shift 4

    run_testbench "$planner" "$params_file" "$world_name" "End of the risk sweep tests" \
        is_risk_sweep_tests:=true \
        is_step_variation_tests:=false \
        is_risks_variation_tests:=false \
        risk_sweep_step:=0.1 \
        optimal_solution_nb_of_iter:=${nb_of_runs} \
        "$@"
}
