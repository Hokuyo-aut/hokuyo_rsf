from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('hokuyo_rsf'),
        'config'
    )
    
    config_yaml = os.path.join(config_dir, 'hokuyo_rsf.yaml')
    
    debugger_config_dir = os.path.join(
        get_package_share_directory('rsf_debugger'),
        'config'
    )
    debugger_yaml = os.path.join(debugger_config_dir, 'rsf_state_publisher.yaml')
    rviz_conf = os.path.join(config_dir, 'rviz.rviz')

    show_debugger = LaunchConfiguration('show_debugger')
    debugger_only = LaunchConfiguration('debugger_only')  # フラグ名を debugger_only に変更

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_debugger',
            default_value='false',
            description='Whether to display the rsf state debugger'
        ),
        # フラグ名を debugger_only に変更 (default: false)
        DeclareLaunchArgument(
            'debugger_only',
            default_value='false',
            description='If true, sensor node (hokuyo_rsf) will not be launched for debugging with bag files, etc.'
        ),

        # メインのセンサノード (debugger_onlyが「true」のときは UnlessCondition により起動しない)
        Node(
            package='hokuyo_rsf',
            executable='hokuyo_rsf',
            name='hokuyo_rsf',
            parameters=[
                config_yaml,
                {'param_files_dir': config_dir}
            ],
            output='screen',
            condition=UnlessCondition(debugger_only)
        ),

        # デバッグ用UIノード
        Node(
            package='rsf_debugger',
            executable='rsf_state_debugger',
            name='sensor_status_display_node',
            parameters=[debugger_yaml],
            output='screen',
            condition=IfCondition(show_debugger)
        ),

        # 音声警告プレイヤー
        Node(
            package='rsf_debugger',
            executable='audio_warning_player.py',
            name='audio_warning_player',
            output='screen',
            condition=IfCondition(show_debugger)
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_conf],
            output='screen'
        )
    ])