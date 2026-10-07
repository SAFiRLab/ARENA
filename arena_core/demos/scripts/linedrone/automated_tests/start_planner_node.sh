#!/bin/bash
# Starts the planner driven by the testbench (ARENA or a benchmark planner). Called by testbench_node every time the planner has to be (re)started.

source "$(dirname "$(readlink -f "$0")")/terminal_utils.sh"

# The planner is restarted many times during a test. tmux reuses its window, but xterm would leave a window behind each time
[ "$TESTBENCH_TERMINAL" = "xterm" ] && TERMINAL_HOLD=0

# Usage: start_planner_node.sh [<benchmark planner> [<params file>]]
#   no argument: ARENA (linedrone_test_node)
#   moar_3d, rrt_star or spline_nsga2: benchmark planner started in place of ARENA (benchmark_planner_launch.py), with
#   its parameters (config/linedrone/benchmarks/<planner>_params.yaml by default)
if [ -z "$1" ]; then
    run_in_terminal "planner" "ros2 launch arena_core linedrone_test_node_launch.py"
else
    params_arg=""
    [ -n "$2" ] && params_arg="params_file:=$2"
    run_in_terminal "planner" "ros2 launch arena_core benchmark_planner_launch.py algo:=$1 $params_arg"
fi
