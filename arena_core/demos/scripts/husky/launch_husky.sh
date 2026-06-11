#!/bin/bash

ROS_WS_PATH=/home/dev_ws

source /opt/ros/humble/setup.bash
source $ROS_WS_PATH/install/setup.bash

# Take into argument if it's simulation or real robot. If no argument is provided, default to simulation.
USE_SIM_TIME=true
if [ "$1" == "real" ]; then
    USE_SIM_TIME=false
fi

if [ "$USE_SIM_TIME" = true ]; then
    ros2 launch clearpath_robots_sim bringup.launch.py use_sim_time:=$USE_SIM_TIME &
    sleep 2
fi

ros2 run clearpath_robots_sim pure_pursuit_node --ros-args -p use_sim_time:=$USE_SIM_TIME &
sleep 1

echo "Husky simulation utils launched successfully! Launching planning node..."

ros2 launch arena_core husky_test_node_launch.py use_sim_time:=$USE_SIM_TIME
