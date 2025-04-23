from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_core',
            namespace='arena_core',
            executable='costmap_3D_node',
            name='costmap_3D_node',
            output='screen',
            emulate_tty=True
        )
    ])