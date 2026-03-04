
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Unity) clock if true'
        ),

        Node(
            package='arena_core',
            executable='traversability_map_node',
            name='traversability_map_node',
            parameters=[
                '/home/dev_ws/src/arena_core/demos/config/husky/grid_map_filters_config.yaml',
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        Node(
            package='arena_core',
            executable='husky_test_node',
            name='husky_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/husky/husky_problem_params.yaml',
                        {'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
