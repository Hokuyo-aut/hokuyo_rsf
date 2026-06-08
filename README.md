# hokuyo_rsf

Version: ROS2 humble

### ビルド
```
cd colcon_ws/src
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git

# nmea_msgs/msg/gpzda が必要
sudo apt remove ros-humble-nmea-msgs
cd colcon_ws/src
git clone https://github.com/hokuyo-rd-release/nmea_msgs.git
colcon build --packages-select nmea_msgs

# jsk_rviz_plugin が必要
cd colcon_ws/src
git clone -b feat/ros2 https://github.com/hokuyo-rd-release/jsk_visualization.git
colcon build

# hokuyo_rsf をビルド
cd ~/colcon_ws
colcon build --symlink-install --packages-select hokuyo_rsf
source ~/colcon_ws/install/setup.bash
```

### Docker

```shell
cd hokuyo_rsf/docker
# Build Image
docker build --network host -t hokuyo_rsf:release .
# Enter the Container
./run.bash -n hokuyo_rsf_release -s /path/to/your/share_folder
# After you exit the container, the endpoint is generated automatically.
# You can use this script when you want to enter the container again.
~/CONTAINER_NAME.bash
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

出力仕様に関する解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_spec_info_jp/

ソース：https://github.com/Hokuyo-aut/hokuyo_navigation2