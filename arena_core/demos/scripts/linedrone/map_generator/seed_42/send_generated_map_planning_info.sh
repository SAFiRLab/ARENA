#!/bin/bash
# Sends the drone position to linedrone_test_node, then a planning request, for a generated map.
#
# Usage: send_generated_map_planning_info.sh [map]
#   map   Name of a map of ressources/generated_map (searched in its subfolders) or path to its .bt file.
#         Default: cluttered_map_seed42 (ressources/generated_map/seed_42).
#
# The drone position and the goal come from config/linedrone/testbench_configs/<map>.yaml, written by the map
# generator when testbench.enabled is true: both are inside the map and at least testbench.clearance m away
# from the obstacles. They are checked against the map bounds saved in the metadata next to the .bt file.
#
# The map has to be published first (publish_generated_map.sh) and linedrone_test_node running
# (ros2 launch arena_core linedrone_test_node_launch.py).

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEMOS_DIR="${SCRIPT_DIR%/scripts/*}"
GENERATED_MAP_DIR="$DEMOS_DIR/ressources/generated_map"
TESTBENCH_CONFIGS_DIR="$DEMOS_DIR/config/linedrone/testbench_configs"
ROS_WS_PATH=${ROS_WS_PATH:-${DEMOS_DIR%/src/*}}
PLANNER_NAMESPACE=/linedrone_test_node

MAP_NAME=$(basename "${1:-cluttered_map_seed42}" .bt)

# The metadata is saved next to the .bt file
BT_FILE=$(find "$GENERATED_MAP_DIR" -name "$MAP_NAME.bt" 2>/dev/null | head -n 1)
METADATA_FILE="${BT_FILE%.bt}.yaml"
if [ -z "$BT_FILE" ]; then
    echo "Map not found: $MAP_NAME.bt in $GENERATED_MAP_DIR"
    exit 1
fi
TESTBENCH_FILE="$TESTBENCH_CONFIGS_DIR/$MAP_NAME.yaml"

for FILE in "$METADATA_FILE" "$TESTBENCH_FILE"; do
    if [ ! -f "$FILE" ]; then
        echo "Not found: $FILE"
        echo "Regenerate the map with testbench.enabled: true in the generator config"
        exit 1
    fi
done

# Reads the drone position and the goal, and checks that they are inside the map. Prints "x y z x y z".
POSITIONS=$(python3 - "$METADATA_FILE" "$TESTBENCH_FILE" <<'EOF'
import sys
import yaml

with open(sys.argv[1]) as f:
    map_config = yaml.safe_load(f)['config']['map_generator']['map']
with open(sys.argv[2]) as f:
    testbench = yaml.safe_load(f)

axes = ('x', 'y', 'z')
low = [float(map_config['origin'][a]) for a in axes]
high = [low[i] + float(map_config['size'][a]) for i, a in enumerate(axes)]
drone = [float(testbench['drone']['position'][a]) for a in axes]
goal = [float(testbench['planning_goal']['position'][a]) for a in axes]

for name, point in (('drone position', drone), ('goal', goal)):
    if any(not low[i] <= point[i] <= high[i] for i in range(3)):
        sys.exit('The {} {} is outside the map [{}, {}]'.format(name, point, low, high))

print(*drone, *goal)
EOF
) || exit 1

read -r DRONE_X DRONE_Y DRONE_Z GOAL_X GOAL_Y GOAL_Z <<< "$POSITIONS"

echo "Map:            $MAP_NAME"
echo "Drone position: ($DRONE_X, $DRONE_Y, $DRONE_Z)"
echo "Goal:           ($GOAL_X, $GOAL_Y, $GOAL_Z)"

source /opt/ros/humble/setup.bash
source "$ROS_WS_PATH/install/setup.bash"

# Wait for linedrone_test_node to subscribe, so the messages are not lost.
# Order matters: planning_activation resets the goal, so the goal is sent last.
ros2 topic pub -w 1 --once $PLANNER_NAMESPACE/drone_pose geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: $DRONE_X, y: $DRONE_Y, z: $DRONE_Z}
}" > /dev/null || exit 1

ros2 topic pub -w 1 --once $PLANNER_NAMESPACE/planning_activation std_msgs/msg/Bool "data: true" > /dev/null || exit 1

ros2 topic pub -w 1 --once $PLANNER_NAMESPACE/goal_pose geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}
}" > /dev/null || exit 1

echo "Planning request sent"
