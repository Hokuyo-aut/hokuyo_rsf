# hokuyo_rsf

### ビルド
```
cd colcon_ws
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git
cd colcon_ws
colcon build --symlink-install
```
### 実行
```shell
ros2 launch hokuyo_rsf hokuyo_rsf.launch.py
```