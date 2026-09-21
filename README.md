# thouzer_gnss_navigation_viewer
GNSSナビゲーション時にサウザーの状態をRViz上に表示するためのリポジトリ  
緯度経度はサウザー自体をbrokerとしてMQTT通信で送信されるデータをSubする。  
サウザーがない場合は、自分でmosquittoを使ってbrokerを立ち上げて、MQTT通信をする。    

# Build 
```bash
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/docker
$ docker build -t gnss-docker::latest .
```
ビルドが完了すれば本パッケージをビルドする   
```bash
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/docker
$ ./run.sh
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select thouzer_gnss_navigation_viewer
$ source ~/ros2_ws/install/setup.bash
```

# Usage
```bash
# サウザーのbrokerを使わない場合は下記を実行する
$ mosquitto -v
# 別のターミナル開いて
$ ros2 launch thouzer_gnss_navigation_viewer
# 別のターミナルを開いて
$ cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/thouzer_gnss_navigation_viewer
$ python3 test_mqtt_cmd_pub.py
```


# CSV waypoint表示（MQTT不要）

`csv_waypoint_viewer` はCSVの経路だけをRVizへ配信します。既存のMQTT版 `gnss_viewer` とは独立し、MQTT接続・ロボットメッシュ・RVizの自動起動を行いません。実機とFlatlandで同じノードを使えます。

```bash
ros2 launch thouzer_gnss_navigation_viewer csv_waypoints.launch.py \
  csv_file:=$HOME/git/GnssNav/csv/101.csv \
  origin_latitude_deg:=36.0834944 \
  origin_longitude_deg:=140.0766774 \
  origin_ellipsoid_height_m:=0.0 \
  frame_id:=map
```

上記の原点は今回の市役所地図用です。実機では使用中の地図の原点に合わせてください。原点の緯度・経度は必須です。高度を持たないCSVなので、全点を原点と同じ楕円体高と仮定してENUへ変換します。方位は東=0度、反時計回りが正です。

CSVヘッダーは `緯度,経度,方位,pause`。`pause` はTrue/False。UTF-8 BOM付きCSVにも対応します。起動時に全行を検証し、不正な行は行番号とともにエラーにします。CSV更新後は表示ノードを再起動してください。CSVは変更しません。

RVizの **Add → MarkerArray** で `/gnss/waypoint_markers` を選び、**Durability Policy=Transient Local、Reliability Policy=Reliable** にします。Fixed Frameは `map`。通常点は赤、停止点は黄色、接続線は水色、矢印はCSVの方位を示します。`show_numbers:=true` で1始まりの番号を表示できます。

表示高さは点・矢印0.3 m、線0.15 m、番号1.0 mで、地形の標高ではありません。全件を固定IDで配信し、ノードを起動したままであればRVizを後から開いても表示できます。全件の置き換えにDELETEALLを使うため、このtopicは専用にしてください。同時に複数のCSV表示ノードを使う場合はtopicを別名へremapし、RVizも別のMarkerArray表示にします。

ROS環境での確認：

```bash
colcon test --packages-select thouzer_gnss_navigation_viewer
colcon test-result --verbose
ros2 topic info /gnss/waypoint_markers --verbose
```

RVizを後から開いて83点（101.csv）が見えること、先頭と40点目が黄色であること、表示ノードを再起動しても重複しないことを確認してください。先頭位置は市役所原点で約 `(90.584, -31.740)` mです。
