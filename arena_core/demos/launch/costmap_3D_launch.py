from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_core',
            executable='costmap_3D_node',
            name='costmap_3D_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/costmap_3D_params.yaml'],
            output='screen',
            emulate_tty=True
        )
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