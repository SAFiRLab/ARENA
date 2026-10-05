import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCRIPTS_DIR = '/home/dev_ws/src/arena_core/demos/scripts/linedrone/automated_tests'
TESTBENCH_CONFIGS_DIR = '/home/dev_ws/src/arena_core/demos/config/linedrone/testbench_configs'

# Planning goal used when it is neither given as launch arguments nor in the testbench config of the map
DEFAULT_GOAL = {'x': 30.0, 'y': 20.0, 'z': 35.0}

# Planner driven by the testbench
PLANNER_NAMESPACE = '/linedrone_test_node'
PLANNER_NODE_NAME = PLANNER_NAMESPACE + '/linedrone_test_node'


def resolve_planning_goal(context):
    """Planning goal from the goal_x/y/z arguments, otherwise from the testbench config of map_name, otherwise DEFAULT_GOAL."""
    goal = dict(DEFAULT_GOAL)

    map_name = LaunchConfiguration('map_name').perform(context)
    config_file = os.path.join(TESTBENCH_CONFIGS_DIR, map_name + '.yaml')
    if os.path.isfile(config_file):
        with open(config_file) as f:
            config = yaml.safe_load(f) or {}
        position = config.get('planning_goal', {}).get('position', {})
        goal.update({axis: float(position[axis]) for axis in goal if axis in position})
        print('[testbench_launch] Planning goal from {}'.format(config_file))
    else:
        print('[testbench_launch] No testbench config for map "{}" ({})'.format(map_name, config_file))

    for axis in goal:
        value = LaunchConfiguration('goal_' + axis).perform(context)
        if value != '':
            goal[axis] = float(value)

    print('[testbench_launch] Planning goal: x={x}, y={y}, z={z}'.format(**goal))
    return {'planning_goal.position.' + axis: value for axis, value in goal.items()}


def generate_launch_description():
    # Launch argument name -> (node parameter name, default value, type)
    args = {
        'world_name': ('world_name', 'undefined', str),
        'output_folder': ('output_folder', '/home/dev_ws/data_output/navigation_3d', str),

        'is_step_variation_tests': ('is_step_variation_tests', 'false', bool),
        'is_hyperparameter_variation_tests': ('is_hyperparameter_variation_tests', 'false', bool),
        'is_optimal_solution_per_objective_tests': ('is_optimal_solution_per_objective_tests', 'false', bool),
        'is_risks_variation_tests': ('is_risks_variation_tests', 'true', bool),

        'coeff_steps': ('step_variation_tests.coeff_steps', '0.0', float),
        'step_variation_nb_of_iter': ('step_variation_tests.nb_of_iter', '0.0', float),

        'hyperparameter_name': ('hyperparameters_variation_tests.hyperparameter_name', 'nb_of_generations', str),
        'hyperparameter_steps': ('hyperparameters_variation_tests.hyperparameter_steps', '0.0', float),
        'hyperparameter_min': ('hyperparameters_variation_tests.hyperparameter_min', '0.0', float),
        'hyperparameter_max': ('hyperparameters_variation_tests.hyperparameter_max', '0.0', float),
        'hyperparameter_nb_of_iter': ('hyperparameters_variation_tests.nb_of_iter', '0.0', float),

        'optimal_solution_nb_of_iter': ('optimal_solution_per_objective_tests.nb_of_iter', '0.0', float),

        # Planner hyperparameters kept fixed during the hyperparameters variation tests, -1 keeps the planner's own value
        'fixed_nb_of_generations': ('fixed_hyperparameters.nb_of_generations', '-1.0', float),
        'fixed_population_size': ('fixed_hyperparameters.population_size', '-1.0', float),
        'fixed_nurbs_sample_size': ('fixed_hyperparameters.nurbs_sample_size', '-1.0', float),
        'fixed_rrt_range': ('fixed_hyperparameters.rrt_range', '-1.0', float),
        'fixed_cost_time': ('fixed_hyperparameters.cost_time', '-1.0', float),
        'fixed_cost_safety': ('fixed_hyperparameters.cost_safety', '-1.0', float),
        'fixed_cost_energy': ('fixed_hyperparameters.cost_energy', '-1.0', float),

        'planner_node_name': ('planner.node_name', PLANNER_NODE_NAME, str),
        'planner_start_command': ('planner.start_command', SCRIPTS_DIR + '/start_planner_node.sh', str),
        'planner_kill_command': ('planner.kill_command', "pkill -INT -f 'lib/arena_core/[l]inedrone_test_node'", str),
        'planner_startup_timeout': ('planner.startup_timeout', '30.0', float),
    }

    return LaunchDescription([
        # Args
        DeclareLaunchArgument('map_name', default_value='ksl_airport_2'),
        *[DeclareLaunchArgument(name, default_value=default) for name, (_, default, _) in args.items()],
        # Planning goal, empty to use the planning_goal of the testbench config of map_name
        *[DeclareLaunchArgument('goal_' + axis, default_value='') for axis in DEFAULT_GOAL],

        # Testbench Utilities Publisher Node
        #Node(
        #    package='arena_core',
        #    executable='testbench_utilities_publisher_node',
        #    name='testbench_utilities_publisher_node',
        #    parameters=[{
        #        'testbench_config_file': PathJoinSubstitution([
        #            '/home/dev_ws/src/arena_core/demos/config/linedrone/testbench_configs',
        #            [LaunchConfiguration('map_name'), '.yaml']
        #        ]),
        #        'octomap_file': PathJoinSubstitution([
        #            '/home/dev_ws/src/arena_core/demos/ressources/saved_octomaps',
        #            [LaunchConfiguration('map_name'), '.bt']
        #        ]),
        #    }],
        #    output='screen',
        #    emulate_tty=True
        #),

        # Testbench Publisher Node
        OpaqueFunction(function=lambda context: [Node(
            package='arena_core',
            executable='testbench_node',
            name='testbench_node',
            parameters=[{
                param: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (param, _, value_type) in args.items()
            }, resolve_planning_goal(context)],
            remappings=[
                ('/navigation/planning_activated', PLANNER_NAMESPACE + '/planning_activation'),
                ('/navigation/goal', PLANNER_NAMESPACE + '/goal_pose'),
                ('/navigation/path_planning_finished', PLANNER_NAMESPACE + '/path_planning_finished'),
                ('/navigation/nurbs_infos', PLANNER_NAMESPACE + '/nurbs_infos'),
                ('/navigation/inflated_octomap', '/navigation/inflated_octomap/full'),
            ],
            output='screen',
            emulate_tty=True
        )]),
    ])
