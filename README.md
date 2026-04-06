# hokuyo_rsf

### ビルド
```
cd colcon_ws/src
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git
cd colcon_ws

# nmea_msgs/msg/gpzda が必要
sudo apt remove ros-humble-nmea-msgs
git pull https://github.com/hokuyo-rd-release/nmea_msgs.git
colcon build --packages-select nmea_msgs
colcon build --symlink-install --packages-select hokuyo_rsf
```
### 実行
```shell
ros2 launch hokuyo_rsf hokuyo_rsf.launch.py
```
