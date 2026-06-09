#!/bin/bash

# Take into argument if it's simulation or real robot. If no argument is provided, default to simulation.
if [ "$1" == "real" ]; then
  echo "Starting Foxglove bridge for real robot..."
  ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=false
else
  echo "Starting Foxglove bridge for simulation..."
  ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true
fi
