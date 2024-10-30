#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
from paho.mqtt import client as mqtt_client
from typing import Dict
from .config import MQTTParam

class MqttHandler:
    debug = True
    def __init__(self, on_message_callback,topic_sub=None,client_id=None,debug=False):
        self.debug = debug
        # brokerに接続時のコールバック関数
        def on_connect(client, userdata, flags, rc):
            if self.debug: print(f"rc: {rc}")
            if rc == 0:
                if self.debug: print("Connected to MQTT Broker!")
                if topic_sub != None:
                    self.subscribe_mqtt(topic_sub)
                else:
                    self.subscribe_mqtt(MQTTParam.topic_nav)
            else:
                if self.debug: print("Failed to connect, return code %d\n", rc)
            
        # brokerからメッセージを受け取ったときのコールバック関数
        def on_message(client, userdata, message):
            payload = message.payload.decode()
            if self.debug: print(f"Received `{payload}` from `{message.topic}` topic")
            on_message_callback(payload)
        if client_id == None:
            self.client = mqtt_client.Client(client_id=MQTTParam.client_id)
        else:
            self.client = mqtt_client.Client(client_id=client_id)
        self.client.username_pw_set(MQTTParam.username, MQTTParam.password) 
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.connect(MQTTParam.broker, MQTTParam.port)
    
    # 指定したトピックをサブスクライブする関数
    def subscribe_mqtt(self, topic: str) -> None:
        self.client.subscribe(topic)
        if self.debug: print(f"Subscribed to topic `{topic}`")
    
    def publish_mqtt_msg(self, topic: str, msg: Dict) -> int:
        result = self.client.publish(topic, json.dumps(msg))
        status = result[0]
        return status
    
    def start_whisperer(self) -> None:
        self.client.loop_start()
        time.sleep(0.5)
    
    def start_nav(self) -> None:
        msg = {"app": "app-whisperer"}  
        topic = MQTTParam.topic_exec
        status = self.publish_mqtt_msg(topic, msg)
        if status != 0:
            print(f"Failed to start whisperer to topic {MQTTParam.topic_exec}")

    def pub_pos(self,serial: str, map: str, pos: float, last: int, lat_deg: float, lon_deg: float, yaw_deg: float) -> None:
        msg = {"serialId": serial, \
               "data": { \
                        "application": "whisperer", \
                        "status": "run", \
                        "map": map, \
                        "alarmRatio": 0.0, \
                        "alertRatio": 0.0, \
                        "pos": round(pos,2), \
                        "last": last, \
                        "listCount": 1, \
                        "listNum": 0, \
                        "listPos": int(pos), \
                        "listLast": last,\
                        "LatLonYaw": { \
                        "lat_deg": round(lat_deg,10), "lon_deg": round(lon_deg,10), "yaw_deg": round(yaw_deg,1) }\
                        }, \
                    "pos": "",\
                    "time": ""
                }
        topic = MQTTParam.topic_event
        self.publish_mqtt_msg(topic, msg)

    def pub_waypoints(self, waypoints):
        for waypoint in waypoints:
            lat_deg, lon_deg, yaw_deg, is_pause = waypoint
            status = "paused" if is_pause else "waypoint" 
            msg = {
                "data": {
                    "application": "whisperer",
                    "status": status,
                    "LatLonYaw": {
                        "lat_deg": round(lat_deg, 10),
                        "lon_deg": round(lon_deg, 10),
                        "yaw_deg": round(yaw_deg, 1)
                    }
                }
            }
        topic = MQTTParam.topic_event
        self.publish_mqtt_msg(topic, msg)

    def pub_success(self,) -> None:
        msg = {
            "app": "#success",
            "running": "OK",
            "work": "OK",
            "MT_teach": False,
            "MT_suspend": False,
            "timestamp": ""
        }
        topic = MQTTParam.topic_status
        self.publish_mqtt_msg(topic, msg)
        
    def avoid_obstacle(self, l: float, theta: float) -> None:
        msg = {"distance_m": str(l), "direction_deg": str(theta)}  
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)
        if status != 0: print(f"Failed to avoid obstacle, send message to topic {MQTTParam.topic_nav}")
        
    def pause_memorize(self) -> None:
        msg = {"app": "pause-memorize"}
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)

    def start_pause(self) -> None:
        msg = {"app": "#pause"}
        topic = MQTTParam.topic_exec
        status = self.publish_mqtt_msg(topic, msg)

    def start_gnss_memorize(self) -> None:
        msg = {"app": "start-gnss-memorize"}  
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)
        
    def stop_gnss_memorize(self) -> None:
        msg = {"app": "stop-gnss-memorize"}  
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)

    def start_gnss_navigate(self) -> None:
        self.start_nav()
        time.sleep(0.5)
        msg = {"app": "start-gnss-navigate"}  
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)
                
    def stop_gnss_navigate(self) -> None:
        msg = {"app": "stop-gnss-navigate"}  
        topic = MQTTParam.topic_nav
        status = self.publish_mqtt_msg(topic, msg)

    def stop_app(self) -> None:
        msg = {"app": ""}  
        topic = MQTTParam.topic_exec
        status = self.publish_mqtt_msg(topic, msg)
        if status != 0:
            print(f"Failed to stop whisperer to topic {MQTTParam.topic_exec}")
        time.sleep(0.5)

    def stop_whisperer(self) -> None:
        self.stop_app()
        self.client.loop_stop()
    
    def change_speed_mode(self, mode) -> None:
        msg = {"app": "#speedmode_" + str(mode)}  
        print(msg)
        topic = MQTTParam.topic_exec
        status = self.publish_mqtt_msg(topic, msg)