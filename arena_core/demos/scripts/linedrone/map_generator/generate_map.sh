#!/bin/bash
# Generates random cluttered 3D maps as octomaps (.bt) in ressources/generated_map.
#
# Usage: generate_map.sh [--build] [map_generator options]
#   --build   Build arena_core before generating (needed after changing the generator code)
#
# Examples:
#   generate_map.sh                                  # Map of config/linedrone/map_generator/map_generator_params.yaml
#   generate_map.sh -s 7                             # Same config, seed 7
#   generate_map.sh -s 0 -n 10                       # 10 maps, seeds 0 to 9
#   generate_map.sh -c my_config.yaml --name forest  # Another config, saved as forest_seed<seed>.bt
#   generate_map.sh --help                           # All the options

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEMOS_DIR="${SCRIPT_DIR%/scripts/*}"
ROS_WS_PATH=${ROS_WS_PATH:-${DEMOS_DIR%/src/*}}

if [ "$1" = "--build" ]; then
    shift
    (cd "$ROS_WS_PATH" && colcon build --packages-select arena_core --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS2=ON) || exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROS_WS_PATH/install/setup.bash"

ros2 run arena_core map_generator "$@"
