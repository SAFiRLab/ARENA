#!/bin/bash

source "$(dirname "$(readlink -f "$0")")/../../terminal_utils.sh"

# Environment name var, only used to name the report
# Possible env_name: A386, L1493, LP1, LP2, CD, CL
env_name="ksl_airport"

num_iter=3
coeff_step=0.05

# Goal position: 
x=-47.547691960948164
y=-11.498129752480677
z=8.028

run_in_terminal "costmap_3D" "ros2 launch arena_core costmap_3D_launch.py"

# The testbench starts and restarts the planner by itself
sleep 1.0
run_in_terminal "testbench" "ros2 launch arena_core testbench_launch.py \
    map_name:=ksl_airport_1 \
    world_name:=${env_name} \
    is_hyperparameter_variation_tests:=false \
    is_step_variation_tests:=true \
    coeff_steps:=${coeff_step} \
    step_variation_nb_of_iter:=${num_iter} \
    goal_x:=${x} \
    goal_y:=${y} \
    goal_z:=${z}"

follow_testbench_nodes
