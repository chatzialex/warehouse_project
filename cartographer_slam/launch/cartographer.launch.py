from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    start_rviz = LaunchConfiguration("start_rviz")

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'
    rviz_config_file = os.path.join(get_package_share_directory('cartographer_slam'), 'config', 'mapper_rviz_config.rviz')

    return LaunchDescription([
      DeclareLaunchArgument(
        "start_rviz",
        default_value="True"
      ),

      Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
      ), 
      Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
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