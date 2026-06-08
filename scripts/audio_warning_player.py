#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class AudioWarningPlayer(Node):
    def __init__(self):
        super().__init__('audio_warning_player')
        self.subscription = self.create_subscription(
            String,
            '/spel/audio_warning',
            self.listener_callback,
            10)
        
        # ステータス文字列と再生する音声ファイルのパスのマッピング
        # 使用環境に合わせて mp3 ファイルのパスを書き換えてください
        self.sound_map = {
            "cpu_overload":       "/home/share/sounds/cpu_overload.mp3",
            "cpu_normal":         "/home/share/sounds/cpu_normal.mp3",
            "temperature_error":  "/home/share/sounds/temp_error.mp3",
            "temperature_normal": "/home/share/sounds/temp_normal.mp3",
            "gnss_lost":          "/home/share/sounds/gnss_lost.mp3",
            "gnss_recovered":     "/home/share/sounds/gnss_recovered.mp3"
        }
        self.get_logger().info('Audio Warning Player Node has been started.')

    def listener_callback(self, msg):
        status = msg.data
        self.get_logger().info(f'Received alert status: "{status}"')
        
        if status in self.sound_map:
            file_path = self.sound_map[status]
            if os.path.exists(file_path):
                self.get_logger().info(f'Playing sound: {file_path}')
                # mpg123 コマンドを使用してバックグラウンドで再生 (sudo apt install mpg123 が必要)
                subprocess.Popen(["mpg123", "-q", file_path])
            else:
                self.get_logger().error(f'Sound file not found at: {file_path}')
        else:
            self.get_logger().warn(f'No sound mapped for status: "{status}"')

def main(args=None):
    rclpy.init(args=args)
    audio_warning_player = AudioWarningPlayer()
    rclpy.spin(audio_warning_player)
    audio_warning_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
