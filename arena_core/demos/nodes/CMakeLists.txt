add_executable(costmap_3D_node costmap_3D_node.cpp)
target_link_libraries(costmap_3D_node ${PROJECT_NAME}_arena_core)
ament_target_dependencies(costmap_3D_node 
    rclcpp 
    octomap_msgs
    sensor_msgs
    tf2_ros
    tf2
    visualization_msgs
)

add_executable(nurbs_test_node nurbs_test_node.cpp ${ARENA_DEMOS_SRCS})
target_link_libraries(nurbs_test_node ${PROJECT_NAME}_arena_core)
ament_target_dependencies(nurbs_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
)

add_executable(mpc_controller_node mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node ${PROJECT_NAME}_arena_core)
ament_target_dependencies(mpc_controller_node 
    rclcpp
    visualization_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2
)

add_executable(linedrone_test_node linedrone_test_node.cpp ${ARENA_DEMOS_SRCS})
target_link_libraries(linedrone_test_node ${PROJECT_NAME}_arena_core)
ament_target_dependencies(linedrone_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    std_msgs
    geometry_msgs
    octomap_msgs
)

install(TARGETS
  costmap_3D_node
  nurbs_test_node
  mpc_controller_node
  linedrone_test_node
  DESTINATION lib/${PROJECT_NAME}
)
