# hokuyo_spel_master_node

### 使い方 (Master node)

```shell
ros2 launch hokuyo_spel_master hokuyo_spel_master_node.launch.py
```



### 使い方 (ROS node)

```shell
ros2 launch hokuyo_spel_master hokuyo_spel_ros_node.launch.py
```

ROSノードの起動時にSPEL Masterにデータストリーミングの開始のコマンドを送信します。

ROSノードとの接続が切れると、Masterは自動でデータストリーミングをオフにします。

MasterとROSノードはデータのパースの関係上、いくつかのファイルを共有しています。ただしユーザーに公開するのはROSノードだけだと想定されますので、実際にはROSノードはMasterと切り離してパッケージを作ることになります。



### 設定

**config**内にMasterとROSノードの設定用のyamlファイルがあります。

Masterのyaml内では、どの情報をストリームするかも選べます。



### SPEL Masterへのコマンドの送り方

ROSノードが起動していることが前提です（実際にSPEL Masterと通信するのはROSノードだけです）。

```shell
ros2 run hokuyo_spel_master send_uint8_command 1
```

- 1: データストリーミング開始
- 2: データストリーミング終了
- 3: RSFセンサ計測開始　#岡本
- 4: RSFセンサ計測終了　#岡本
- 5: RSFセンサ計測値リセット (4と3を続けて送るのとほぼ同じ挙動)　#岡本

SPEL Masterは3~5のコマンドを受け取った後、それぞれに対応したstd_msgs::msg::Empty ("/rsf_start, /rsf_stop, /rsf_reset")のデータをパブリッシュします。


### IPアドレスの変更方法

```shell
ros2 run hokuyo_spel_master send_ip_address 192.168.10.100
```

SPEL Masterはコマンドを受け取った後、std_msgs::msg::String（"/spel_ip_address"）のデータをパブリッシュします。

不正なIPを受け取ったときはIPアドレスはパブリッシュせず、ROSノードにWarningを返します。
