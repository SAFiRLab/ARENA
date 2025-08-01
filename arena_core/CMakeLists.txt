cmake_minimum_required(VERSION 3.3.2)
project(arena_core)

# Enable testing via command-line flag (e.g. -DENABLE_TESTING=ON)
option(ENABLE_TESTING "Enable tests" OFF)
# Enable coverage via command-line flag (e.g. -DENABLE_COVERAGE=ON)
option(ENABLE_COVERAGE "Enable code coverage" OFF)
# Enable or disable ROS2 support via command-line flag (e.g. -DENABLE_ROS2=ON)
option(ENABLE_ROS2 "Build with ROS2 support" OFF)


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
find_package(Pagmo REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)

if (ENABLE_ROS2)
    find_package(rclcpp REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(octomap_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
endif()


include_directories(
    libs/arena_core/include
    libs/arena_core/tests/include
    demos/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_subdirectory(libs)

add_subdirectory(demos)

# Install launch files.
if (ENABLE_ROS2)
    install(DIRECTORY
    demos/launch
    DESTINATION share/${PROJECT_NAME}/
    )
endif()

ament_package()
