#!/bin/bash

source "$(dirname "$(readlink -f "$0")")/../../terminal_utils.sh"

# Environment name var, only used to name the report
# Possible env_name: A386, L1493, LP1, LP2, CD, CL
env_name="CL"

#hyperparameter_name="nb_of_generations"
#hyperparameter_name="population_size"
hyperparameter_name="nurbs_sample_size"
hyperparameter_steps=2
hyperparameter_min=11
hyperparameter_max=251
num_iter=10

run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py"

# The testbench starts and restarts the planner by itself
sleep 1.0
run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
    map_name:=map_test_3 \
    world_name:=${env_name} \
    is_hyperparameter_variation_tests:=true \
    is_step_variation_tests:=false \
    hyperparameter_name:=${hyperparameter_name} \
    hyperparameter_min:=${hyperparameter_min} \
    hyperparameter_max:=${hyperparameter_max} \
    hyperparameter_steps:=${hyperparameter_steps} \
    hyperparameter_nb_of_iter:=${num_iter}"

follow_testbench_nodes
