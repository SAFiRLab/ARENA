ros2 topic pub /linedrone_test_node/planning_activation std_msgs/msg/Bool "data: true" --once


ros2 topic pub /linedrone_test_node/goal_pose geometry_msgs/msg/PointStamped "{
  header: {
    frame_id: 'map'
  },
  point: {
    x: 59.367,
    y: -26.573,
    z: 44.997
  }
}" --once