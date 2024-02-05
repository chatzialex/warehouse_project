import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanner_rviz_config.rviz')

    
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rviz",
            default_value="True"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True"),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            namespace='/',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/robot/cmd_vel')]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='/',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='/',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=[('/cmd_vel', '/robot/cmd_vel')]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='/',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]),
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
                                        'bt_navigator']}]),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(start_rviz))
    ])