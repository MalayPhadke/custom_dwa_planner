from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    model = LaunchConfiguration('model')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'rviz',
        'dwa_planner_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='burger'),
        DeclareLaunchArgument('x_pos', default_value='-2.0'),
        DeclareLaunchArgument('y_pos', default_value='-0.5'),
        DeclareLaunchArgument('z_pos', default_value='0.0'),
        DeclareLaunchArgument('ns', default_value='local_planner'),

        # Parameter files
        DeclareLaunchArgument('dwa_param', default_value=PathJoinSubstitution([
            FindPackageShare('custom_dwa_planner'), 'config', 'params.yaml'
        ])),

        # Basic parameters
        DeclareLaunchArgument('hz', default_value='20.0'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('subscribe_count_th', default_value='3'),
        DeclareLaunchArgument('sleep_time_after_finish', default_value='0.5'),
        DeclareLaunchArgument('v_path_width', default_value='0.05'),
        DeclareLaunchArgument('use_footprint', default_value='false'),
        DeclareLaunchArgument('use_path_cost', default_value='false'),
        DeclareLaunchArgument('use_scan_as_input', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Topic remaps
        DeclareLaunchArgument('cmd_vel', default_value='/cmd_vel'),
        DeclareLaunchArgument('local_map', default_value='/local_map'),
        DeclareLaunchArgument('local_goal', default_value='/move_base_simple/goal'),
        DeclareLaunchArgument('odom', default_value='/odom'),
        DeclareLaunchArgument('dist_to_goal_th', default_value='/dist_to_goal_th'),
        DeclareLaunchArgument('scan', default_value='/scan'),
        DeclareLaunchArgument('footprint', default_value='/footprint'),
        DeclareLaunchArgument('path', default_value='/path'),
        DeclareLaunchArgument('target_velocity', default_value='/target_velocity'),


        # dwa_planner
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('custom_dwa_planner'),
        #             'launch', 'local_planner.launch.py'
        #         )
        #     ),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'use_scan_as_input': 'true',
        #         'v_path_width': '0.02'
        #     }.items()
        # ),

        # Main node
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner_node',
            namespace=LaunchConfiguration('ns'),
            name='dwa_planner',
            parameters=[
                LaunchConfiguration('dwa_param'),
                {
                    'HZ': LaunchConfiguration('hz'),
                    'GLOBAL_FRAME': LaunchConfiguration('global_frame'),
                    'SUBSCRIBE_COUNT_TH': LaunchConfiguration('subscribe_count_th'),
                    'SLEEP_TIME_AFTER_FINISH': LaunchConfiguration('sleep_time_after_finish'),
                    'V_PATH_WIDTH': LaunchConfiguration('v_path_width'),
                    'USE_FOOTPRINT': LaunchConfiguration('use_footprint'),
                    'USE_PATH_COST': LaunchConfiguration('use_path_cost'),
                    'USE_SCAN_AS_INPUT': LaunchConfiguration('use_scan_as_input'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('cmd_vel', LaunchConfiguration('cmd_vel')),
                ('local_map', LaunchConfiguration('local_map')),
                ('move_base_simple/goal', LaunchConfiguration('local_goal')),
                ('odom', LaunchConfiguration('odom')),
                ('dist_to_goal_th', LaunchConfiguration('dist_to_goal_th')),
                ('scan', LaunchConfiguration('scan')),
                ('footprint', LaunchConfiguration('footprint')),
                ('path', LaunchConfiguration('path')),
                ('target_velocity', LaunchConfiguration('target_velocity')),
            ]
        ),


        # AMCL for localization (provides map->odom transform)  
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': True,
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'alpha5': 0.2,
                'base_frame_id': 'base_footprint',
                'beam_skip_distance': 0.5,
                'beam_skip_error_threshold': 0.9,
                'beam_skip_threshold': 0.3,
                'do_beamskip': False,
                'global_frame_id': 'map',
                'lambda_short': 0.1,
                'laser_likelihood_max_dist': 2.0,
                'laser_max_range': 100.0,
                'laser_min_range': -1.0,
                'laser_model_type': 'likelihood_field',
                'max_beams': 60,
                'max_particles': 2000,
                'min_particles': 500,
                'odom_frame_id': 'odom',
                'pf_err': 0.05,
                'pf_z': 0.99,
                'recovery_alpha_fast': 0.0,
                'recovery_alpha_slow': 0.0,
                'resample_interval': 1,
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'save_pose_rate': 0.5,
                'sigma_hit': 0.2,
                'tf_broadcast': True,
                'transform_tolerance': 1.0,
                'update_min_a': 0.2,
                'update_min_d': 0.25,
                'z_hit': 0.5,
                'z_max': 0.05,
                'z_rand': 0.5,
                'z_short': 0.05,
                'scan_topic': 'scan'
            }]
        ),

        # Map server (provides map frame)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_dir,
                'topic_name': 'map',
                'frame_id': 'map'
            }]
        ),

        # Lifecycle manager to activate map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
