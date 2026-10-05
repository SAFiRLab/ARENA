from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        # Args
        DeclareLaunchArgument('map_name', default_value='map_test_3'),

        # Testbench Utilities Publisher Node
        Node(
            package='arena_core',
            executable='testbench_utilities_publisher_node',
            name='testbench_utilities_publisher_node',
            parameters=[{
                'testbench_config_file': PathJoinSubstitution([
                    '/home/dev_ws/src/arena_core/demos/config/linedrone/testbench_configs',
                    [map_name, '.yaml']
                ]),
                'octomap_file': PathJoinSubstitution([
                    '/home/dev_ws/src/arena_core/demos/ressources/saved_octomaps',
                    [map_name, '.bt']
                ]),
            }],
            output='screen',
            emulate_tty=True
        )
    ])
