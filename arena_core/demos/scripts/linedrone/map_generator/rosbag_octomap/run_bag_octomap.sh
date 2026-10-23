#!/bin/bash
# Octomap of the lidar of a ROS 1 bag: octomap_server fed by bag_to_octomap.py (velodyne clouds deskewed + laser scans,
# ray-cast from the sensor pose of the bag TF), then saved. Sensor model of the simulated lidar map
# (config/linedrone/map_generator/scaffold_power_line_params.yaml, lidar_mapping.octomap).
#
# Usage: run_bag_octomap.sh <bag> <output .bt> [ROS domain] [bag_to_octomap.py options...]
#   ROS domain   default 84, isolated from the other ROS sessions
# Environment: RESOLUTION (default 0.25), MISS (default 0.48), LOG_DIR (logs and sensor poses, default /tmp/bag_octomap)
# Needs the rosbags Python package (pip install rosbags). Only the processes started here are stopped at the end.

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
DEMOS_DIR="${SCRIPT_DIR%/scripts/*}"
ROS_WS_PATH=${ROS_WS_PATH:-${DEMOS_DIR%/src/*}}
BAG=$(readlink -f "$1"); OUT_BT=$(readlink -f "$2"); shift 2
export ROS_DOMAIN_ID=${1:-84}; shift
LOG_DIR=${LOG_DIR:-/tmp/bag_octomap}
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$ROS_WS_PATH/install/setup.bash"

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done; }
trap cleanup EXIT

# The node itself (not ros2 run, whose child would survive the cleanup); latch off: only /octomap_binary is published
"$(ros2 pkg prefix octomap_server)/lib/octomap_server/octomap_server_node" --ros-args -p resolution:=${RESOLUTION:-0.25} \
    -p frame_id:=map -p latch:=false -p sensor_model.hit:=0.7 -p sensor_model.miss:=${MISS:-0.48} \
    -p sensor_model.min:=0.12 -p sensor_model.max:=0.97 -p sensor_model.max_range:=40.0 > "$LOG_DIR/octomap_server.log" 2>&1 &
PIDS+=($!)

set -o pipefail
python3 "$SCRIPT_DIR/bag_to_octomap.py" "$BAG" --poses-output "$LOG_DIR/poses.csv" "$@" 2>&1 | tee "$LOG_DIR/replay.log" || exit 1
rm -f "$OUT_BT"
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:="$OUT_BT" > "$LOG_DIR/saver.log" 2>&1
[ -f "$OUT_BT" ] || { echo "octomap not saved, see $LOG_DIR/saver.log"; exit 1; }
echo "$OUT_BT saved"
