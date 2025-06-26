from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # RViz configuration file path
    rviz_config_dir = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'rviz',
        'dwa_planner_rviz_odom.rviz')
    
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='local_planner'),

        # Parameter files
        DeclareLaunchArgument('dwa_param', default_value=PathJoinSubstitution([
            FindPackageShare('dwa_planner'), 'config', 'dwa_param.yaml'
        ])),
        DeclareLaunchArgument('robot_param', default_value=PathJoinSubstitution([
            FindPackageShare('dwa_planner'), 'config', 'robot_param.yaml'
        ])),

        # Basic parameters
        DeclareLaunchArgument('hz', default_value='20.0'),
        DeclareLaunchArgument('global_frame', default_value='odom'),
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

        # Main node
        Node(
            package='dwa_planner',
            executable='dwa_planner_node',
            namespace=LaunchConfiguration('ns'),
            name='dwa_planner',
            parameters=[
                LaunchConfiguration('dwa_param'),
                LaunchConfiguration('robot_param'),
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
        
        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )
    ])
