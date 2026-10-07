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

add_executable(linedrone_test_node linedrone/linedrone_test_node.cpp ${ARENA_DEMOS_LINEDRONE_SRCS})
target_link_libraries(linedrone_test_node ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(linedrone_test_node 
    rclcpp 
    visualization_msgs
    tf2_ros
    tf2
    std_msgs
    geometry_msgs
    nav_msgs
    octomap_msgs
    grid_map_core
    arena_msgs
)

# Benchmark planners compared to ARENA (testbench/benchmarks), same ROS interface as linedrone_test_node
set(ARENA_BENCHMARKS_COMMON_SRCS
    ${ARENA_DEMOS_LINEDRONE_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/testbench/benchmarks/common/baseline_path_scorer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/testbench/benchmarks/common/benchmark_planner_node.cpp
)

set(ARENA_BENCHMARKS_TARGETS moar_3d_node rrt_star_node spline_nsga2_node)
add_executable(moar_3d_node
    testbench/benchmarks/moar_3d/moar_3d_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/testbench/benchmarks/moar_3d/weighted_astar_3d.cpp
    ${ARENA_BENCHMARKS_COMMON_SRCS}
)
add_executable(rrt_star_node testbench/benchmarks/rrt_star/rrt_star_node.cpp ${ARENA_BENCHMARKS_COMMON_SRCS})
add_executable(spline_nsga2_node testbench/benchmarks/spline_nsga2/spline_nsga2_node.cpp ${ARENA_BENCHMARKS_COMMON_SRCS})

foreach(target ${ARENA_BENCHMARKS_TARGETS})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/testbench/benchmarks)
    target_link_libraries(${target} ${PROJECT_NAME}_arena_core ${OCTOMAP_LIBRARIES} yaml-cpp)
    ament_target_dependencies(${target}
        rclcpp
        visualization_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        octomap_msgs
        grid_map_core
        arena_msgs
    )
endforeach()

add_executable(testbench_node linedrone/testbench_nodes/testbench_node.cpp)
target_link_libraries(testbench_node ${OCTOMAP_LIBRARIES})
ament_target_dependencies(testbench_node 
    rclcpp 
    visualization_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    octomap_msgs
    arena_msgs
)

add_executable(testbench_utilities_publisher_node
    linedrone/testbench_nodes/testbench_utilities_publisher_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../data/linedrone/src/testbench_config.cpp
)
target_link_libraries(testbench_utilities_publisher_node ${OCTOMAP_LIBRARIES} yaml-cpp)
ament_target_dependencies(testbench_utilities_publisher_node 
    rclcpp 
    nav_msgs
    octomap_msgs
)

add_executable(map_generator linedrone/map_generator/map_generator.cpp ${ARENA_DEMOS_MAP_GENERATOR_SRCS})
target_link_libraries(map_generator ${OCTOMAP_LIBRARIES} yaml-cpp)
# Not used directly, but grid_map_core sets global Eigen plugin definitions that need its headers
ament_target_dependencies(map_generator grid_map_core)

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
  linedrone_test_node
  testbench_node
  testbench_utilities_publisher_node
  map_generator
  husky_test_node
  traversability_map_node
  ${ARENA_BENCHMARKS_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)
