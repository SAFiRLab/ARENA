#!/bin/bash

source "$(dirname "$(readlink -f "$0")")/../../../terminal_utils.sh"
source "$(dirname "$(readlink -f "$0")")/../map_settings.sh"

# Distance between the nodes of the RRT used to initialize the population, in meters (default: 5.0)
hyperparameter_name="rrt_range"
hyperparameter_steps=1.0
hyperparameter_min=1.0
hyperparameter_max=30.0
num_iter=10

# Inflating the generated map takes about 1 min, the testbench waits for the costmap before the first planning
run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py bt_file:=${bt_file}"

# The testbench starts and restarts the planner by itself
sleep 1.0
run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
    map_name:=${map_name} \
    world_name:=${world_name} \
    is_hyperparameter_variation_tests:=true \
    is_step_variation_tests:=false \
    hyperparameter_name:=${hyperparameter_name} \
    hyperparameter_min:=${hyperparameter_min} \
    hyperparameter_max:=${hyperparameter_max} \
    hyperparameter_steps:=${hyperparameter_steps} \
    hyperparameter_nb_of_iter:=${num_iter}"

follow_testbench_nodes
