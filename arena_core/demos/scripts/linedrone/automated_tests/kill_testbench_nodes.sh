#!/bin/bash
# Stops every node started by the automated tests scripts (costmap, testbench and planner), whatever the terminal mode.

source "$(dirname "$(readlink -f "$0")")/terminal_utils.sh"

stop_testbench_nodes
