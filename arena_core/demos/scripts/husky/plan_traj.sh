#!/bin/bash

GLOBAL_FRAME_ID=world


ros2 topic pub /planning_activation std_msgs/msg/Bool "data: true" --once

sleep 1

ros2 topic pub /goal_pose geometry_msgs/msg/PointStamped "{
  header: {
    frame_id: \"$GLOBAL_FRAME_ID\"
  },
  point: {
    x: 1020.0,
    y: -650.0,
    z: 7.0
  }
}" --once
