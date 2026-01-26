from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('hokuyo_spel_master'),
    'config'
  )

  master_config_yaml = os.path.join(config_dir, 'hokuyo_spel_master_config.yaml')

  return LaunchDescription([
    Node(
      package='hokuyo_spel_master',
      executable='hokuyo_spel_master_node',
      name='hokuyo_spel_master_node',
      parameters=[
        master_config_yaml,
        {'param_files_dir': config_dir}
      ],
      output='screen'
    )
  ])
