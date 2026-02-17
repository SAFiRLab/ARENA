add_executable(costmap_3D_node linedrone/costmap_3D_node.cpp ${ARENA_DEMOS_LINEDRONE_SRCS})
target_link_libraries(costmap_3D_node ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(costmap_3D_node 
    rclcpp 
    octomap_msgs
    sensor_msgs
    tf2_ros
    tf2
    visualization_msgs
    grid_map_core
)

add_executable(nurbs_test_node nurbs_test_node.cpp ${ARENA_DEMOS_LINEDRONE_SRCS})
target_link_libraries(nurbs_test_node ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(nurbs_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    grid_map_core
)

add_executable(mpc_controller_node xmaxx/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(mpc_controller_node 
    rclcpp
    visualization_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2
    grid_map_core
)

add_executable(linedrone_test_node linedrone/linedrone_test_node.cpp ${ARENA_DEMOS_LINEDRONE_SRCS})
target_link_libraries(linedrone_test_node ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(linedrone_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    std_msgs
    geometry_msgs
    octomap_msgs
    grid_map_core
)

add_executable(husky_test_node husky/husky_test_node.cpp ${ARENA_DEMOS_HUSKY_SRCS})
target_link_libraries(husky_test_node ${PROJECT_NAME}_arena_core yaml-cpp)
ament_target_dependencies(husky_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    std_msgs
    geometry_msgs
    grid_map_core
    grid_map_pcl
    grid_map_msgs
    grid_map_ros
)

add_executable(traversability_map_node husky/traversability_map_node.cpp ${ARENA_DEMOS_HUSKY_SRCS})
target_link_libraries(traversability_map_node ${PROJECT_NAME}_arena_core yaml-cpp)
ament_target_dependencies(traversability_map_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    std_msgs
    geometry_msgs
    grid_map_core
    grid_map_pcl
    grid_map_ros
    grid_map_msgs
    pcl_conversions
    grid_map_filters
    grid_map_cv
    filters
)

install(TARGETS
  costmap_3D_node
  nurbs_test_node
  mpc_controller_node
  linedrone_test_node
  husky_test_node
  traversability_map_node
  DESTINATION lib/${PROJECT_NAME}
)
