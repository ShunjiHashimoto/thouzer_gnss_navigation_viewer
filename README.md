# thouzer_gnss_navigation_viewer

GNSSナビゲーションの情報をRVizに表示するパッケージです。用途は次の2つです。

| 用途                      | 入力                            | 表示する内容                         | MQTT | RVizの起動                   |
| ------------------------- | ------------------------------- | ------------------------------------ | ---- | ---------------------------- |
| 1. 走行中のサウザーを表示 | GnssNavが配信するMQTTメッセージ | 現在位置・向き、受信したwaypointなど | 必要 | 専用launchが起動             |
| 2. CSVのwaypointを表示    | `101.csv` などのCSV           | waypointの位置・向き・接続線・停止点 | 不要 | 既存のRVizを使うか、別途起動 |

用途2は実機がなくても利用でき、ロボットの現在位置は表示しません。Flatlandと併用する場合、ロボットはFlatland側で表示し、このパッケージはwaypointを重ねて表示します。

## 共通の準備・ビルド

ROS 2とRVizが利用できる環境に、このリポジトリを `~/ros2_ws/src/thouzer_gnss_navigation_viewer` として配置します。Dockerを使う場合は、末尾の「補足：Docker環境」を先に参照してください。

ROS 2の環境を読み込んだ端末で、以下を実行します。**ビルドは両方式で共通です。用途ごとにビルドし直す必要はありません。** ソースの更新後は再ビルドしてください。

```bash
cd ~/ros2_ws
colcon build --base-paths src/thouzer_gnss_navigation_viewer --packages-select thouzer_gnss_navigation_viewer
source ~/ros2_ws/install/setup.bash
```

新しい端末を開いたときも、ROS 2の環境に加えて `source ~/ros2_ws/install/setup.bash` を実行してください。

## 1. MQTTで走行中のサウザーを表示する

### 接続先を確認する

GnssNavが配信する位置・waypointのメッセージを、MQTTブローカ経由で受信します。次の設定を実際の機体・環境に合わせてください。

- `thouzer_gnss_navigation_viewer/config.py`：接続先ブローカ、ポート、認証情報、機体ID、購読topic。
- `thouzer_gnss_navigation_viewer/gnss_navigation_viewer.py` の `initial_blh`：表示座標の原点。

MQTT方式の原点は現在コード内で設定しています。用途2のコマンドライン引数はMQTT方式には適用されません。設定ファイルを変更した場合は、共通のビルド手順で反映します。MQTT方式には `paho-mqtt` も必要です。

### 起動する

サウザー側などで稼働しているブローカへ接続でき、GnssNavがメッセージを配信している状態で実行します。

```bash
ros2 launch thouzer_gnss_navigation_viewer gnss_navigation_viewer.launch.py
```

MQTT表示ノードと専用設定のRVizが起動します。このコマンドは可視化用で、GNSSナビゲーションの走行開始操作はGnssNav側で行います。

### 実機なしで表示を確認する場合

ローカルのブローカを使う場合は、上記の接続先設定をローカル環境に合わせ、別端末で起動します。

```bash
mosquitto -v
```

MQTT方式のviewerを起動後、別端末でテスト送信スクリプトを実行します。送信側の接続設定も同じブローカ・topicに合わせてください。

```bash
cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/thouzer_gnss_navigation_viewer
python3 test_mqtt_cmd_pub.py
```

## 2. MQTTなしでCSVのwaypointを表示する

CSV表示専用ノードは、指定したファイルを起動時に読み込んで表示します。MQTTへの接続やRVizの自動起動は行いません。

### CSVと原点を指定して起動する

以下は、市役所地図に `101.csv` を重ねる例です。CSVのパスは実際の配置先に合わせてください。

```bash
ros2 launch thouzer_gnss_navigation_viewer csv_waypoints.launch.py \
  csv_file:=$HOME/git/GnssNav/csv/101.csv \
  origin_latitude_deg:=36.0834944 \
  origin_longitude_deg:=140.0766774 \
  show_numbers:=true
```

原点は、表示する地図の `(x=0, y=0)` に対応する緯度・経度です。地図に重ねる場合は、その地図の原点と一致させてください。CSVの先頭点を自動で原点にはしません。JSONファイルは不要です。

| 引数                          | 意味                            | 省略時    |
| ----------------------------- | ------------------------------- | --------- |
| `csv_file`                  | waypointのCSVパス               | 必須      |
| `origin_latitude_deg`       | 原点の緯度［度］                | 必須      |
| `origin_longitude_deg`      | 原点の経度［度］                | 必須      |
| `origin_ellipsoid_height_m` | 原点の楕円体高［m］             | `0.0`   |
| `frame_id`                  | 表示の座標系                    | `map`   |
| `show_numbers`              | 1始まりの点番号を表示           | `false` |
| `use_sim_time`              | ROSのシミュレーション時刻を使用 | `false` |

高さを持たないCSVなので、各点は原点と同じ楕円体高と仮定してENUへ変換します。方位は東=0度、反時計回りが正です。

### RVizに表示を追加する

RVizが起動していなければ、別端末で起動します。

```bash
rviz2
```

起動中のRVizで **Panels → Displays → Add → By display type → MarkerArray** を選び、以下を設定します。

| 項目                          | 値                                                     |
| ----------------------------- | ------------------------------------------------------ |
| Global Options → Fixed Frame | `map`（`frame_id` を変更した場合はそれに合わせる） |
| 追加したMarkerArray → Topic  | `/gnss/waypoint_markers`                             |
| Durability Policy             | `Transient Local`                                    |
| Reliability Policy            | `Reliable`                                           |

通常点は赤、停止点は黄色、接続線は水色、矢印はCSVの方位を示します。点・矢印は高さ0.3 m、線は0.15 m、番号は1.0 mに表示します。この高さは見やすさのためのもので、地形の標高ではありません。

市役所の例で先頭が見えない場合は、**Panels → Views** で `TopDownOrtho` を選び、X=`90.584`、Y=`-31.740`、Scale=`50` にすると先頭付近を確認できます。

### Flatlandと併用する場合

Flatlandは普段のコマンドで起動し、viewerは別端末で上記のCSV表示コマンドに `use_sim_time:=true` を追加して起動します。表示先はFlatlandが起動したRVizを使います。

`101.csv` の先頭にロボットを置く場合は、Flatland側の既存引数を次の値にします。

```text
initial_pose_x:=90.584
initial_pose_y:=-31.740
initial_pose_a:=-0.1557193095
```

地図・仮想GNSS・viewerで同じ原点を使ってください。別PC・コンテナで動かす場合は、ROS_DOMAIN_IDなどのROS通信設定も合わせます。

画面操作で追加したRViz設定を保存する場合は、**File → Save Config As** で `~/gnss_simulation.rviz` などリポジトリ外に保存できます。

### CSVの形式と更新

- ヘッダー：`緯度,経度,方位,pause`。`pause` は `True` / `False`。
- UTF-8 BOM付きCSVにも対応します。不正な行は行番号とともにエラーにします。
- CSVの内容を変更したら表示ノードを再起動します。viewerがCSVを書き換えることはありません。
- ノードを起動したままであれば、RVizを後から開いても表示できます。
- 全件の置き換えにDELETEALLを使うため、`/gnss/waypoint_markers` は専用topicとして使います。複数のCSV表示ノードを同時に使う場合はtopicを別名へremapし、RVizにも別のMarkerArray表示を追加します。

### 表示の確認

`101.csv` では83点が表示され、先頭と40点目が黄色になります。表示ノードを再起動しても点が重複しないことを確認してください。

```bash
ros2 topic info /gnss/waypoint_markers --verbose
```

開発時のテストは以下で実行します。

```bash
colcon test --packages-select thouzer_gnss_navigation_viewer
colcon test-result --verbose
```

## 補足：Docker環境

すでにROS 2とRVizが使える場合、この準備は不要です。付属のDocker環境を使う場合は、ホスト側でイメージを作成してコンテナを起動します。

```bash
cd ~/ros2_ws/src/thouzer_gnss_navigation_viewer/docker
docker build -t gnss-docker:latest .
./run.sh
```

`docker/run.sh` のワークスペースのマウント元は `/home/ubuntu/ros2_ws` に固定されています。別の配置では実際のパスに合わせてください。コンテナ内では `/root/ros2_ws` にマウントされます。コンテナ起動後は、冒頭の「共通の準備・ビルド」へ進みます。画面表示にはホスト側のX11環境が必要です。
