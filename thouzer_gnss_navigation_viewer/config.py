import random

use_case_robot = "thouzer_cross" # icart3 or thouzer_cross
class MQTTParam:
    broker = '192.168.212.1' # メッセージの仲介を行う、PublisherからSubscriberｈメッセージを転送する
    # broker = "localhost"
    port = 1883 # デフォルトのport番号
    if use_case_robot == "icart3":
        serial = "RMS-TFRG-000"
    elif use_case_robot == "icartmini":
        serial = "RMS-TFRG-003"
    elif use_case_robot == "thouzer_cross":
        serial = "RMS-SZE2-000"
    topic_exec = "0/THOUZER_HW/"+serial+"/exec/cmd"
    topic_nav = "0/WHISPERER/"+serial+"/nav"
    topic_pos = "0/WHISPERER/"+serial+"/pos2D_DWO"
    topic_event = "0/THOUZER_HW/"+serial+"/event/app"
    topic_status = "0/WHISPERER/"+serial+"/app_status"

    client_id = f'publish-mqtt-{random.randint(0, 1000)}'
    username = 'mqtt'
    password = 'sI7G@DijuY'
