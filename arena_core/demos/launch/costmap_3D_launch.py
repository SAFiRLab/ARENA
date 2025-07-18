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
        ),
        # Launch Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'resolution': 0.5,
                'frame_id': 'world',
                'sensor_model/max_range': 15.0,
                'sensor_model/min_range': 1.0
                #'sensor_model/hit_prob': 0.7,
                #'sensor_model/miss_prob': 0.4,
                #'sensor_model/unknown_prob': 0.5,
                #'map_load_path': '',  # disable loading from file
            }]
        )
    ])