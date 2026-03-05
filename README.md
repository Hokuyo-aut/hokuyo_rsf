# hokuyo_rsf

### 使い方

```shell
ros2 launch hokuyo_rsf hokuyo_rsf.launch.py
```

### SPEL Masterへのコマンドの送り方

ROSノードが起動していることが前提です（実際にSPEL Masterと通信するのはROSノードだけです）。

```shell
ros2 run hokuyo_rsf send_uint8_command 1
```
- 1: データストリーミング開始
- 2: データストリーミング終了
- 3: RSFセンサ計測開始
- 4: RSFセンサ計測終了
- 5: RSFセンサ計測値リセット (4と3を続けて送るのとほぼ同じ挙動)

SPEL Masterは3~5のコマンドを受け取った後、それぞれに対応したstd_msgs::msg::Empty ("/rsf_start, /rsf_stop, /rsf_reset")のデータをパブリッシュします。