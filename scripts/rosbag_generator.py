import os
import rclpy
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rosbag2_py

def create_mcap_bag():
    # 生成するバッグのフォルダ名
    bag_path = 'rsf_verification_bag'
    
    if os.path.exists(bag_path):
        import shutil
        shutil.rmtree(bag_path)

    writer = rosbag2_py.SequentialWriter()
    
    # 環境に合わせて 'mcap' または 'sqlite3' を選択してください
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    writer.open(storage_options, converter_options)

    # 1. 提示されたYAMLパラメータに基づくトピック名の登録
    topic_templates = [
        ('/fix', 'sensor_msgs/msg/NavSatFix'),
        ('/rsf/rsf_fix_state', 'std_msgs/msg/String'),
        ('/rsf/rsf_fix_type', 'std_msgs/msg/String'),
        ('/gga', 'nmea_msgs/msg/Gpgga'),
        ('/rsf/rsf_odom_type', 'std_msgs/msg/String'),
        ('/rsf/rsf_odom_state', 'std_msgs/msg/String'),
        ('/rsf/rsf_odom', 'nav_msgs/msg/Odometry'),
        ('/rsf/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray')
    ]

    for name, msg_type in topic_templates:
        topic_info = rosbag2_py.TopicMetadata(
            name=name, type=msg_type, serialization_format='cdr'
        )
        writer.create_topic(topic_info)

    # 2. タイムライン設定 (10Hzで30秒間 = 300ステップ)
    start_time_ns = 1000000000
    duration_sec = 30
    hz = 10
    dt_ns = int(1e9 / hz)

    print("新しいトピック名で検証用バッグファイルを生成中...")

    for step in range(duration_sec * hz):
        current_time_ns = start_time_ns + (step * dt_ns)
        current_sec = (current_time_ns - start_time_ns) / 1e9

        # --- デフォルト値（正常系）のセット ---
        nav_sat_fix = NavSatFix()
        nav_sat_fix.position_covariance[0] = 0.02**2  # Accuracy: 0.02m (緑)
        gpgga = Gpgga()
        gpgga.gps_qual = 4                            # RTK Fix (緑)
        gnss_state = "good"
        gnss_type = "GNSS (switch)"                   # 緑
        odom_type = "GNSS (switch)"                   # 緑
        odom_state = "good"                             # 緑
        
        publish_lio = True 
        cpu_val = "45"
        temp_val = "50.5"

        # =================================================================
        # 5秒ごとの状態切り替えロジック
        # =================================================================
        if current_sec < 5.0:
            # 【Phase 1: オールグリーン (正常系)】
            pass

        elif current_sec < 10.0:
            # 【Phase 2: 精度低下】
            nav_sat_fix.position_covariance[0] = 0.35**2  # Accuracy: 0.35m (水色)
            gpgga.gps_qual = 5                            # RTK Float (黄)
            gnss_state = "be careful"             # 黄
            gnss_type = "GNSS raw"                        # 黄
            odom_type = "LIO raw"                         # 黄
            odom_state = "be careful"                        # 黄

        elif current_sec < 15.0:
            # 【Phase 3: GNSS完全消失 (LOST)】 -> "gnss_lost"
            nav_sat_fix.position_covariance[0] = float('nan') 
            gpgga.gps_qual = 0                                
            gnss_state = "abnormal"                    
            gnss_type = "LIO (gnss is abnormal)"              
            odom_type = "LIO (gnss is abnormal)"              
            odom_state = "abnormal"                           

        elif current_sec < 20.0:
            # 【Phase 4: GNSSが復帰 (RECOVERED) ＆ LIO周波数低下】 -> "gnss_recovered"
            nav_sat_fix.position_covariance[0] = 0.01**2      
            gpgga.gps_qual = 4                                
            # 3回に1回しか配信しない（周波数を約3.3Hzに落として赤判定に）
            if step % 4 != 0:
                publish_lio = False

        elif current_sec < 25.0:
            # 【Phase 5: CPU高負荷】 -> "cpu_overload"
            cpu_val = "95"  # limit(90) 超え

        else:
            # 【Phase 6: デバイスオーバーヒート】 -> "cpu_normal" & "temperature_error"
            cpu_val = "40"
            temp_val = "82.3"  # limit(75.0) 超え

        # =================================================================
        # 書き込み処理
        # =================================================================
        writer.write('/fix', serialize_message(nav_sat_fix), current_time_ns)
        writer.write('/gga', serialize_message(gpgga), current_time_ns)

        msg_gnss_state = String(data=gnss_state)
        msg_gnss_type = String(data=gnss_type)
        msg_odom_type = String(data=odom_type)
        msg_odom_state = String(data=odom_state)
        
        writer.write('/rsf/rsf_fix_state', serialize_message(msg_gnss_state), current_time_ns)
        writer.write('/rsf/rsf_fix_type', serialize_message(msg_gnss_type), current_time_ns)
        writer.write('/rsf/rsf_odom_type', serialize_message(msg_odom_type), current_time_ns)
        writer.write('/rsf/rsf_odom_state', serialize_message(msg_odom_state), current_time_ns)

        if publish_lio:
            odom_msg = Odometry()
            writer.write('/rsf/rsf_odom', serialize_message(odom_msg), current_time_ns)

        diag_array = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "spel_device"
        status.values = [
            KeyValue(key="cpu_usage", value=cpu_val),
            KeyValue(key="device_temperature", value=temp_val)
        ]
        diag_array.status.append(status)
        writer.write('/rsf/diagnostics', serialize_message(diag_array), current_time_ns)

    print(f"完了！ '{bag_path}' が生成されました。")

if __name__ == '__main__':
    rclpy.init()
    create_mcap_bag()
    rclpy.shutdown()
