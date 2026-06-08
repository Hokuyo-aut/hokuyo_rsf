from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('hokuyo_rsf'),
    'config'
  )
  
  config_yaml = os.path.join(config_dir, 'hokuyo_rsf.yaml')
  debugger_yaml = os.path.join(config_dir, 'rsf_state_publisher.yaml')
  rviz_conf = os.path.join(config_dir, 'rviz.rviz')

  show_debugger = LaunchConfiguration('show_debugger')

  return LaunchDescription([
    DeclareLaunchArgument(
      'show_debugger',
      default_value='true',
      description='Whether to display the rsf state debugger'
    ),
    Node(
      package='hokuyo_rsf',
      executable='hokuyo_rsf',
      name='hokuyo_rsf',
      parameters=[
        config_yaml,
        {'param_files_dir': config_dir}
      ],
      output='screen'
    ),
    Node(
      package='hokuyo_rsf',
      executable='rsf_state_debugger',
      name='sensor_status_display_node',
      parameters=[debugger_yaml],
      output='screen',
      condition=IfCondition(show_debugger)
    ),
    Node(
      package='hokuyo_rsf',
      executable='audio_warning_player.py',
      name='audio_warning_player',
      output='screen',
      condition=IfCondition(show_debugger)
    ),
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_conf],
        output='screen'
    )
  ])
