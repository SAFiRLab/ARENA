#!/bin/bash
# Publishes the octomap of a map and every path of an ARENA front (publish_risk_scenarios.py, MarkerArray latched and
# republished every 2 s, frame "map"): /risk_scenarios/map (voxels shaded by height, dark edges) and
# /risk_scenarios/paths (one namespace per solution, magenta = lowest energy to green = lowest safety cost, white
# spheres at the start and the goal). View in Foxglove (foxglove_bridge) or RViz, fixed frame "map". Ctrl-C to stop.
#
# Usage: publish_front.sh [map] [front folder] [publish_risk_scenarios.py options...]
#   map            name in publish_risk_scenarios.py OCTOMAPS (default scaffold_power_line_real)
#   front folder   solutions.csv + paths.csv, relative to this folder (default <map>/arena_front_infl0)
# Uses the ROS_DOMAIN_ID of the shell (same as the viewer).

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEMOS_DIR="${SCRIPT_DIR%/data/*}"
ROS_WS_PATH=${ROS_WS_PATH:-${DEMOS_DIR%/src/*}}
MAP=${1:-scaffold_power_line_real}
FRONT=${2:-$MAP/arena_front_infl0}
shift $(( $# < 2 ? $# : 2 ))

source /opt/ros/humble/setup.bash
source "$ROS_WS_PATH/install/setup.bash"
cd "$SCRIPT_DIR" || exit 1
exec python3 publish_risk_scenarios.py --map "$MAP" --front "$FRONT" --start-goal --height-shading --voxel-edges "$@"
