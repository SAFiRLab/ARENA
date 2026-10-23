from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Analysis only: every solution of the report written with its costs and its clearance (empty: no export)
        DeclareLaunchArgument('export_solutions_file', default_value='',
                              description='optimization.export_solutions_file of the planner (CSV, appended)'),
        Node(
            package='arena_core',
            executable='linedrone_test_node',
            name='linedrone_test_node',
            namespace='linedrone_test_node',
            parameters=['/home/dev_ws/src/arena_core/demos/config/linedrone/linedrone_problem_params.yaml',
                        {'optimization.export_solutions_file': LaunchConfiguration('export_solutions_file')}],
            output='screen'
        )
    ])
