#!/bin/bash

source "$(dirname "$(readlink -f "$0")")/../../../terminal_utils.sh"
source "$(dirname "$(readlink -f "$0")")/../map_settings.sh"

num_iter=3
coeff_step=0.05

# Inflating the generated map takes about 1 min, the testbench waits for the costmap before the first planning
run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py bt_file:=${bt_file}"

# The testbench starts and restarts the planner by itself.
# The start and the goal come from the testbench config of map_name.
sleep 1.0
run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
    map_name:=${map_name} \
    world_name:=${world_name} \
    is_hyperparameter_variation_tests:=false \
    is_step_variation_tests:=true \
    coeff_steps:=${coeff_step} \
    step_variation_nb_of_iter:=${num_iter}"

follow_testbench_nodes
