#!/bin/bash

pkill -f /joint_state_broadcaster
pkill -f joint_state_broadcaster
pkill -f /platform_velocity_controller
pkill -f platform_velocity_controller
pkill -f /husky_test_node
pkill -f husky_test_node
pkill -f /traversability_map_node
pkill -f traversability_map_node
pkill -f /pure_pursuit_node
pkill -f pure_pursuit_node
pkill -f /ros2_control_node
pkill -f ros2_control_node
pkill -f /robot_state_publisher
pkill -f robot_state_publisher
