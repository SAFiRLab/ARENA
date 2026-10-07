from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS_FILE = '/home/dev_ws/src/arena_core/demos/config/linedrone/costmap_3D_params.yaml'


def launch_costmap(context):
    """Costmap node with the params file, the map of the bt_file argument overriding saved_map.bt_file when given."""
    parameters = [PARAMS_FILE]

    bt_file = LaunchConfiguration('bt_file').perform(context)
    if bt_file != '':
        parameters.append({'saved_map.bt_file': bt_file})

    return [
        Node(
            package='arena_core',
            executable='costmap_3D_node',
            name='costmap_3D_node',
            parameters=parameters,
            output='screen',
            emulate_tty=True
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bt_file', default_value='',
                              description='Octomap (.bt) to load. Empty: saved_map.bt_file of costmap_3D_params.yaml'),
        OpaqueFunction(function=launch_costmap)
    ])

"""def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_core',
            namespace='arena_core',
            executable='costmap_3D_node',
            name='costmap_3D_node',
            output='screen',
            emulate_tty=True,
            parameters=['/home/dev_ws/src/arena_core/demos/config/costmap_3D_params.yaml']
        )#,
        # Launch Octomap server
        #Node(
        #    package='octomap_server',
        #    executable='octomap_server_node',
        #    name='octomap_server_node',
        #    output='screen',
        #    emulate_tty=True,
        #    parameters=[{
        #        'resolution': 0.5,
        #        'frame_id': 'map',
        #        #'sensor_model/max_range': 15.0,
        #        #'sensor_model/min_range': 1.0,
        #        'latch': True,
        #        'map_file': '/home/dev_ws/src/arena_core/demos/ressources/saved_octomaps/CL_map_res_50cm.bt',
        #    }]
        #)
    ])"""
