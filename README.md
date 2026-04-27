# hokuyo_rsf

Version: ROS2 humble

### ビルド
```
cd colcon_ws/src
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git
cd colcon_ws

# nmea_msgs/msg/gpzda が必要
sudo apt remove ros-humble-nmea-msgs
git clone https://github.com/hokuyo-rd-release/nmea_msgs.git
colcon build --packages-select nmea_msgs
colcon build --symlink-install --packages-select hokuyo_rsf
```

### Docker

```
cd hokuyo_rsf/docker
docker build --network host -t hokuyo_rsf:release .
./run.bash -n hokuyo_rsf_release -s /path/to/your/share_folder
```

### 実行

```shell
# Node only
ros2 launch hokuyo_rsf hokuyo_rsf.launch.py

# With Visualization
ros2 launch hokuyo_rsf hokuyo_rsf_sample.launch.py
```
### 自律走行サンプル

解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_app_info_jp/

ソース：https://github.com/Hokuyo-aut/hokuyo_navigation2