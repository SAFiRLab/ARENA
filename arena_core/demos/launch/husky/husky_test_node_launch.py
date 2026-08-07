
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if use_sim_time == 'true':
        traversability_map_node = Node(
            package='arena_core',
            executable='traversability_map_node',
            name='traversability_map_node',
            parameters=[
                '/home/dev_ws/src/arena_core/demos/config/husky/grid_map_filters_config.yaml'
            ],
            remappings=[
                ('/pose', '/groundTruth/poseStamped'),
                ('/imu', '/imu')
            ],
            output='screen'
        )

        husky_test_node = Node(
            package='arena_core',
            executable='husky_test_node',
            name='husky_test_node',
            namespace='husky_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/husky/husky_problem_params.yaml',
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/pose', '/groundTruth/poseStamped')
            ],
            output='screen'
        )
    else:
        traversability_map_node = Node(
            package='arena_core',
            executable='traversability_map_node',
            name='traversability_map_node',
            parameters=[
                '/home/dev_ws/src/arena_core/demos/config/husky/grid_map_filters_config.yaml'
            ],
            remappings=[
                ('/pose', '/cartography_pkg/ekf/odometry_filtered'),
                ('/imu', '/imu/data')
            ],
            output='screen'
        )

        husky_test_node = Node(
            package='arena_core',
            executable='husky_test_node',
            name='husky_test_node',
            namespace='husky_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/husky/husky_problem_params.yaml',
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/odometry', '/cartography_pkg/ekf/odometry_filtered'),
            ],
            output='screen'
        )

    return [traversability_map_node, husky_test_node]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Unity) clock if true'
        ),

        OpaqueFunction(function=launch_setup)
    ])
