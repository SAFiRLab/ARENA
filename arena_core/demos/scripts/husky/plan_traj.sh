#!/bin/bash

GLOBAL_FRAME_ID=world


ros2 topic pub /husky_test_node/planning_activation std_msgs/msg/Bool "data: true" --once

sleep 1

ros2 topic pub /husky_test_node/goal_pose geometry_msgs/msg/PointStamped "{
  header: {
    frame_id: \"$GLOBAL_FRAME_ID\"
  },
  point: {
    x: -12.38,
    y: -160.44,
    z: 0.2
  }
}" --once
