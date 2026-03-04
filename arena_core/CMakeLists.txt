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
find_package(Eigen3)
find_package(Pagmo REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)


include_directories(
    libs/arena_core/include
    libs/arena_core/tests/include
    demos/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_subdirectory(libs)

if (ENABLE_ROS2)    
    add_subdirectory(demos)
endif()

# Install launch files.
if (ENABLE_ROS2)
    install(DIRECTORY
    demos/launch
    DESTINATION share/${PROJECT_NAME}/
    )
endif()

ament_package()
