# hokuyo_rsf

Version: ROS2 humble

## セットアップ

### 依存パッケージの導入
```bash
# nmea_msgs (カスタム版が必要な場合)
git clone https://github.com/hokuyo-rd-release/nmea_msgs.git
# jsk_visualization
git clone -b feat/ros2 https://github.com/hokuyo-rd-release/jsk_visualization.git
```

### ビルド
```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select hokuyo_rsf
source install/setup.bash
```

## 実行

### 通常起動（全ノード）
```bash
ros2 launch hokuyo_rsf hokuyo_rsf_sample.launch.py
```

### RViz のみを確認する（ドライバを起動しない）：
```bash
ros2 launch hokuyo_rsf hokuyo_rsf_sample.launch.py use_sensors:=false
```


## rsf_state_debugger ステータス表示の詳細
`rsf_state_debugger` ノードは、RViz 上にシステムのリアルタイムステータスをオーバーレイ表示し、異常時には視覚・音声で警告を行います。

![rsf](Images/rsf_debugger.png)

### ステータス項目一覧
| 項目名 | 説明 | 参照トピック | 詳細説明 |
| :--- | :--- | :--- | :--- |
| **Accuracy** | 推定される GNSS 位置精度（m）。 | `/spel/nav_sat_fix` | **緑**: ≤0.1m, **黄**: ≤4.0m, **赤**: >4.0m |
| **GPGGA Qual** | NMEA の GPS 品質インジケータ。 | `/spel/gpgga` | **緑**: RTK Fix (4), **黄**: RTK Float (5), **シアン**: GPS/DGPS, **赤**: その他 |
| **GPGGA Detail** | 測位品質の詳細説明。 | `/spel/gpgga` | **背景緑**: RTK Fix, **背景黄**: RTK Float, **背景赤**: Invalid |
| **GNSS Type** | 測位データのソース。 | `/spel/switch_fix_type` | Internal / External など |
| **GNSS State** | 測位計算の収束状態。 | `/spel/switch_fix_state` | **緑**: FIX, **黄**: FLOAT, **赤**: その他 |
| **Odometry Type** | 現在の主オドメトリソース。 | `/spel/switch_odom_type` | LIO (switch), GNSS (switch), LIO (raw) など |
| **Odometry State** | システムの安定状態。 | `/spel/switch_odom_state` | **緑**: steady, **黄**: その他 |
| **LIO Rate** | 直近1000サンプルの平均周波数。 | `/spel/lidar_rate_odom` | **緑**: ≥9.5Hz, **黄**: ≥5.0Hz, **赤**: <5.0Hz |
| **CPU Usage** | システム全体の CPU 使用率。 | `/spel/diagnostics` | **緑**: <70%, **黄**: <90%, **赤**: 閾値超過 |
| **Device Temp** | 内部センサーの測定温度。 | `/spel/diagnostics` | **緑**: <55℃, **黄**: <75℃, **赤**: 閾値超過 |

### アラート通知

#### 1. ビジュアルアラート
重大な異常を検知した際、RViz 画面中央に赤色の大きなテキストが表示されます。
- **!!! HIGH CPU LOAD !!!**: CPU 使用率が `cpu_limit`（デフォルト 90%）を超過。
- **!!! DEVICE OVERHEAT !!!**: 温度が `temp_limit`（デフォルト 75.0℃）を超過。

#### 2. 音声警告 (audio_warning_player)
以下のイベントが発生した際、`/spel/audio_warning` トピックを介して音声再生がトリガーされます。
- `cpu_overload` / `cpu_normal`
- `temperature_error` / `temperature_normal`
- `gnss_lost` / `gnss_recovered`


### 自律走行サンプル

解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_app_info_jp/

出力仕様に関する解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_spec_info_jp/

ソース：https://github.com/Hokuyo-aut/hokuyo_navigation2