from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_core',
            executable='husky_test_node',
            name='husky_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/husky_problem_params.yaml'],
            output='screen'
        )
    ])
