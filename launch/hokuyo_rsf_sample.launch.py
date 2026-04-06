from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('hokuyo_rsf'),
    'config'
  )
  
  config_yaml = os.path.join(config_dir, 'hokuyo_rsf.yaml')
  rviz_conf = os.path.join(config_dir, 'rviz.rviz')

  return LaunchDescription([
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
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_conf],
        output='screen'
    )
  ])
