cmake_minimum_required(VERSION 3.0.2)
project(arena_msgs CXX)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
    DIRECTORY msg
    FILES
    Empty.msg
)


