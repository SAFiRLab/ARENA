cmake_minimum_required(VERSION 3.3.2)
project(arena_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
    "msg/Empty.msg"
    "msg/Coordinate.msg"
    "msg/Mission.msg"
    "msg/MissionRisks.msg"
    "msg/OptimizerPathInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
)

ament_package()