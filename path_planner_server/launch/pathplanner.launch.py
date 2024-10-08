import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    keepout_map_file = LaunchConfiguration("keepout_map_file")
    is_elevator_topic_string = LaunchConfiguration("is_elevator_topic_string")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanner_rviz_config.rviz')
    waypoint_follower_yaml = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'waypoint_follower.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')
    keepout_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'maps', keepout_map_file.perform(context))

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_topic)]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=[('/cmd_vel', cmd_vel_topic)]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower_yaml, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml,
                        {'use_sim_time': use_sim_time},
                        {'yaml_filename' : keepout_yaml}]),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'waypoint_follower',
                                        'filter_mask_server',
                                        'costmap_filter_info_server']}]),
        Node(
            package='path_planner_server',
            executable='attach_server',
            name='attach_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {"is_elevator_topic_string" : is_elevator_topic_string}
            ],
            remappings=[('/cmd_vel', cmd_vel_topic)]
        ),
        Node(
            package='path_planner_server',
            executable='go_to_pose_server',
            name='go_to_pose_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_topic)]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(start_rviz))
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rviz",
            default_value="True"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True"),
        DeclareLaunchArgument(
             "keepout_map_file",
            default_value="warehouse_map_sim_keepout.yaml"
        ),
        DeclareLaunchArgument(
             "is_elevator_topic_string",
            default_value="False"
        ),
        DeclareLaunchArgument(
             "cmd_vel_topic",
            default_value="/robot/cmd_vel"
        ),
        OpaqueFunction(function=launch_setup)
    ])