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
  map_file = LaunchConfiguration("map_file")

  map_full_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file.perform(context))
  rviz_config_file = os.path.join(get_package_share_directory('map_server'), 'config', 'mapper_rviz_config.rviz')

  return [
    Node(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time},
                  {'yaml_filename':map_full_path}]),

    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_mapper',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time},
                  {'autostart': True},
                  {'node_names': ['map_server',]}]),

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
        default_value="True"
      ),
      DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml"
      ),
      OpaqueFunction(function=launch_setup)])