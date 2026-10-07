#!/bin/bash
# Publishes a generated map with the costmap_3D node (inflated and SDF octomaps, see costmap_3D_launch.py).
#
# Usage: publish_generated_map.sh [map]
#   map   Path to a .bt file, or name of a map of ressources/generated_map, searched in its subfolders
#         (e.g. cluttered_map_seed42 for ressources/generated_map/seed_42/cluttered_map_seed42.bt).
#         Default: the most recent map of ressources/generated_map.
#
# To plan in this map with the testbench, use map_name:=<map name> so the start and the goal come from
# config/linedrone/testbench_configs/<map name>.yaml (written by the generator when testbench.enabled is true).

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEMOS_DIR="${SCRIPT_DIR%/scripts/*}"
GENERATED_MAP_DIR="$DEMOS_DIR/ressources/generated_map"
ROS_WS_PATH=${ROS_WS_PATH:-${DEMOS_DIR%/src/*}}

if [ -z "$1" ]; then
    BT_FILE=$(find "$GENERATED_MAP_DIR" -name '*.bt' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)
    if [ -z "$BT_FILE" ]; then
        echo "No map in $GENERATED_MAP_DIR, generate one with $SCRIPT_DIR/generate_map.sh"
        exit 1
    fi
elif [ -f "$1" ]; then
    BT_FILE=$(readlink -f "$1")
else
    BT_FILE=$(find "$GENERATED_MAP_DIR" -name "$(basename "${1%.bt}").bt" 2>/dev/null | head -n 1)
    BT_FILE=${BT_FILE:-$GENERATED_MAP_DIR/${1%.bt}.bt}
fi

if [ ! -f "$BT_FILE" ]; then
    echo "Map not found: $BT_FILE"
    exit 1
fi

echo "Publishing $BT_FILE"

source /opt/ros/humble/setup.bash
source "$ROS_WS_PATH/install/setup.bash"

ros2 launch arena_core costmap_3D_launch.py bt_file:="$BT_FILE"
