# thouzer_gnss_navigation_viewer
GNSSナビゲーション時にサウザーの状態をRViz上に表示するためのリポジトリ
緯度経度はサウザー自体をbrokerとしてMQTT通信で送信されるデータをSubする。
サウザーがない場合は、自分でmosquittoを使ってbrokerを立ち上げて、MQTT通信をする。   

# Build 
```bash
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer
$ docker build -t gnss-docker::latest .
```
ビルドが完了すれば本パッケージをビルドする   
```
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/docker
$ ./run.sh
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select thouzer_gnss_navigation_viewer
$ source ~/ros2_ws/install/setup.bash
```

# Usage
```
# サウザーのbrokerを使わない場合は下記を実行する
$ mosquitto -v
# 別のターミナル開いて
$ ros2 launch thouzer_gnss_navigation_viewer
# 別のターミナルを開いて
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/thouzer_gnss_navigation_viewer
$ python3 test_mqtt_cmd_pub.py
```
