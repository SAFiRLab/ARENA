cmake_minimum_required(VERSION 3.3.2)
project(arena_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
    "msg/Empty.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
        std_msgs
        sensor_msgs
        nav_msgs
)

ament_package()