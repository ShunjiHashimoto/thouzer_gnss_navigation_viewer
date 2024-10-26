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
antenna_data = []

with open(gnss_data_file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if(row[5] != "4" or row[14] == "" or row[15] == ""): 
            print("not fix")
            continue
        # 3つのアンテナの緯度と経度および姿勢情報を取得（0:中心, 6:右, 12:左, 18:姿勢）
        antenna_data.append({
            "center": {"lat": float(row[2]), "lon": float(row[3])},
            "right": {"lat": float(row[8]), "lon": float(row[9])},
            "left": {"lat": float(row[14]), "lon": float(row[15])},
            "attitude": {"yaw": float(0.0)}  # 姿勢情報（yawのみ）
        })


# MQTTクライアントのセットアップ
client = mqtt.Client()

# 接続
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# データを順に送信
for data in antenna_data:
    # 送信するデータをJSON形式で作成
    message = {
        "data": {
            "AntennaPositions": {
                "center": {"lat": data["center"]["lat"], "lon": data["center"]["lon"]},
                "right": {"lat": data["right"]["lat"], "lon": data["right"]["lon"]},
                "left": {"lat": data["left"]["lat"], "lon": data["left"]["lon"]}
            },
            "Attitude": {
                "yaw": data["attitude"]["yaw"]
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
