#!/bin/bash

FRAME_ID='world'

# Send either a zero vel cmd, a front 0.5 m/s cmd, a back -0.5 m/s, a turning 0.5 rad/s, or a turning -0.5 rad/s cmd to the platform_velocity_controller.
# Take in an argument for which cmd to send, default to zero vel cmd if no argument is provided.

if [ "$1" == "front" ]; then
    VEL_CMD="linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
elif [ "$1" == "back" ]; then
    VEL_CMD="linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
elif [ "$1" == "turn_left" ]; then
    VEL_CMD="linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"
elif [ "$1" == "turn_right" ]; then
    VEL_CMD="linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}"
else
    VEL_CMD="linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
fi

# Publish at 10hz to the /platform_velocity_controller/cmd_vel topic to send the a TwistStamped to the platform_velocity_controller.
ros2 topic pub /platform_velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    frame_id: \"$FRAME_ID\"
  },
  twist: {
    $VEL_CMD
  }
}" -r 10
