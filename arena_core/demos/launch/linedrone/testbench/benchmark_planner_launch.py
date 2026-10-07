"""Benchmark planner compared to ARENA, started in place of linedrone_test_node.

The planner has the name and the namespace of linedrone_test_node, so the testbench drives it without any change (topics,
parameters). It gets the robot parameters of ARENA (linedrone_problem_params.yaml) and its own parameters
(config/linedrone/benchmarks/<algo>_params.yaml, or params_file).

Usage:
    ros2 launch arena_core benchmark_planner_launch.py algo:=moar_3d|rrt_star|spline_nsga2 [params_file:=<yaml>]
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_DIR = '/home/dev_ws/src/arena_core/demos/config/linedrone'
ROBOT_PARAMS = os.path.join(CONFIG_DIR, 'linedrone_problem_params.yaml')
ALGORITHMS = ['moar_3d', 'rrt_star', 'spline_nsga2']


def launch_planner(context):
    algo = LaunchConfiguration('algo').perform(context)
    if algo not in ALGORITHMS:
        raise RuntimeError('Unknown benchmark planner "{}", choose one of {}'.format(algo, ALGORITHMS))

    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file == '':
        params_file = os.path.join(CONFIG_DIR, 'benchmarks', algo + '_params.yaml')
    print('[benchmark_planner_launch] {} with {}'.format(algo, params_file))

    return [
        Node(
            package='arena_core',
            executable=algo + '_node',
            name='linedrone_test_node',
            namespace='linedrone_test_node',
            parameters=[ROBOT_PARAMS, params_file],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('algo', default_value='moar_3d', description='Benchmark planner: ' + ', '.join(ALGORITHMS)),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Parameters of the planner. Empty: config/linedrone/benchmarks/<algo>_params.yaml'),
        OpaqueFunction(function=launch_planner)
    ])
