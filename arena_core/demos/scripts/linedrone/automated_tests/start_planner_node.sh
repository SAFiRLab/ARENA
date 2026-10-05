#!/bin/bash
# Starts the planner driven by the testbench. Called by testbench_node every time the planner has to be (re)started.

source "$(dirname "$(readlink -f "$0")")/terminal_utils.sh"

# The planner is restarted many times during a test. tmux reuses its window, but xterm would leave a window behind each time
[ "$TESTBENCH_TERMINAL" = "xterm" ] && TERMINAL_HOLD=0

run_in_terminal "planner" "ros2 launch arena_core linedrone_test_node_launch.py"
