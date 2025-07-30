from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_core',
            executable='linedrone_test_node',
            name='linedrone_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/linedrone_problem_params.yaml'],
            output='screen'
        )
    ])
