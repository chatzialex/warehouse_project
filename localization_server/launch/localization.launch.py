import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")

    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_sim.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('localization_server'), 'config', 'localizer_rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rviz",
            default_value="False"
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(start_rviz)
        )
    ])