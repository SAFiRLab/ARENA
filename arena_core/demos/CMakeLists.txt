find_package(octomap REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(octomap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)

include_directories(
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

set(ARENA_DEMOS_LINEDRONE_SRCS
    ${ARENA_DEMOS_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_problem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/linedrone_nurbs_analyzer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/old_nurbs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/nurbs_analyzer_old.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/costmap_3D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linedrone/costmap_mapping.cpp
)

set(ARENA_DEMOS_HUSKY_SRCS
    ${ARENA_DEMOS_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/husky/husky_problem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/husky/husky_nurbs_analyzer.cpp
)

add_subdirectory(nodes)

add_subdirectory(tests)
