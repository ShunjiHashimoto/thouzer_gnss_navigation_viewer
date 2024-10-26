import time
import json
import csv
import paho.mqtt.client as mqtt
from config import MQTTParam

# MQTTサーバーの設定
MQTT_BROKER = MQTTParam.broker  # MQTTブローカのアドレス（Dockerの場合、ホストのIPや'localhost'を指定）
MQTT_PORT = 1883           # MQTTのポート（通常は1883）
thouzer_topic = MQTTParam.topic_event

# CSVファイルの読み込み
gnss_data_file = '../csv/log20241006-103121.csv'
waypoint_data_file = '../csv/waypoint_test.csv'
latlonyaw_data = []
waypoint_list = []

with open(gnss_data_file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if(row[5] != "4" or row[14] == "" or row[15] == ""): 
            # print("not fix")
            continue
        latlonyaw_data.append({
            "status":"run",
            "LatLonYaw": {"lat_deg": float(row[2]), "lon_deg": float(row[3]), "yaw_deg": float(0.0)},
        })

with open(waypoint_data_file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for i, row in enumerate(reader):
        if i==0: continue
        waypoint_list.append({
            "status":"waypoint",
            "LatLonYaw": {"lat_deg": float(row[0]), "lon_deg": float(row[1]), "yaw_deg": float(row[2])},
        })


# MQTTクライアントのセットアップ
client = mqtt.Client()

# 接続
client.connect(MQTT_BROKER, MQTT_PORT, 60)

for waypoint in waypoint_list:
    # 送信するデータをJSON形式で作成
    message = {
        "data": {
            "status" :waypoint["status"],
            "LatLonYaw": {
                "lat_deg": waypoint["LatLonYaw"]["lat_deg"],
                "lon_deg": waypoint["LatLonYaw"]["lon_deg"],
                "yaw_deg": waypoint["LatLonYaw"]["yaw_deg"]
            },
        }
    }

    # JSON文字列に変換
    payload = json.dumps(message)

    # メッセージを指定したトピックにpublish
    client.publish(thouzer_topic, payload)
    print(f"Published: {payload}")
    time.sleep(0.5)
    

# データを順に送信
for data in latlonyaw_data:
    # 送信するデータをJSON形式で作成
    message = {
        "data": {
            "status":data["status"],
            "LatLonYaw": {
                "lat_deg": data["LatLonYaw"]["lat_deg"],
                "lon_deg": data["LatLonYaw"]["lon_deg"],
                "yaw_deg": data["LatLonYaw"]["yaw_deg"]
            }
        }
    }

    # JSON文字列に変換
    payload = json.dumps(message)

    # メッセージを指定したトピックにpublish
    client.publish(thouzer_topic, payload)
    print(f"Published: {payload}")

    # 1秒待機して次のデータを送信
    time.sleep(0.5)

# MQTTクライアントの終了
client.disconnect()
