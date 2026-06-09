#!/bin/bash

ROS_WS_PATH=~/dev_ws

cd $ROS_WS_PATH

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS2=ON
