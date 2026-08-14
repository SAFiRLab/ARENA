from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if use_sim_time == 'true':
        traversability_map_node = Node(
            package='arena_core',
            executable='traversability_map_node',
            name='traversability_map_node',
            parameters=[
                PathJoinSubstitution([FindPackageShare('arena_core'), 'config/husky/grid_map_filters_config.yaml']),
                PathJoinSubstitution([FindPackageShare('arena_core'), 'config/husky/traversability_map_config.yaml'])
            ],
            remappings=[
                ('/pose', '/groundTruth/poseStamped'),
                ('/imu', '/imu')
            ],
            output='screen'
        )
    else:
        traversability_map_node = Node(
            package='arena_core',
            executable='traversability_map_node',
            name='traversability_map_node',
            parameters=[
                PathJoinSubstitution([FindPackageShare('arena_core'), 'config/husky/grid_map_filters_config.yaml']),
                PathJoinSubstitution([FindPackageShare('arena_core'), 'config/husky/traversability_map_config.yaml'])
            ],
            remappings=[
                ('/pose', '/cartography_pkg/ekf/odometry_filtered'),
                ('/imu', '/imu/data')
            ],
            output='screen'
        )

    return [traversability_map_node]

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Unity) clock if true'
        ),

        OpaqueFunction(function=launch_setup)
    ])
