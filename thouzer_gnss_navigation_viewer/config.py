import random

use_case_robot = "thouzer_cross" # icart3 or thouzer_cross
use_case_antenna = "f9p" # f9p or acm
use_case_gnss_correction = "ichimill" # ichimill or geortk

class MQTTParam:
    broker = '192.168.212.1' # メッセージの仲介を行う、PublisherからSubscriberｈメッセージを転送する
    # broker = "localhost"
    port = 1883 # デフォルトのport番号
    if use_case_robot == "icart3":
        serial = "RMS-TFRG-000"
        frontLength_m = 0.48
    elif use_case_robot == "icartmini":
        serial = "RMS-TFRG-003"
        frontLength_m = 0.19
    elif use_case_robot == "thouzer_cross":
        serial = "RMS-SZE2-000"
        frontLength_m = 0.92
    topic_exec = "0/THOUZER_HW/"+serial+"/exec/cmd"
    topic_nav = "0/WHISPERER/"+serial+"/nav"
    topic_pos = "0/WHISPERER/"+serial+"/pos2D_DWO"
    topic_event = "0/THOUZER_HW/"+serial+"/event/app"
    topic_status = "0/WHISPERER/"+serial+"/app_status"

    client_id = f'publish-mqtt-{random.randint(0, 1000)}'
    username = 'mqtt'
    password = 'sI7G@DijuY'

class F9PParam:
    debug = False
    is_write_csv = True
    if use_case_antenna == "f9p":
        ttyUSB0 = "/dev/ttyF9P0"
        ttyUSB1 = "/dev/ttyF9P2"
        ttyUSB2 = "/dev/ttyF9P3"
    elif use_case_antenna == "acm":
        ttyUSB0 = "/dev/ttyACM0"
        ttyUSB1 = "/dev/ttyACM1"
        ttyUSB2 = "/dev/ttyACM2"

class GnssProcessorParam:
    debug = True

class IchimillParam:
    debug = False
    if use_case_gnss_correction == "ichimill":
        ntripUsername="el61oh8o"
        ntripPassword="2sxgrn"
        ntripPort=2101
        ntripUrl="ntrip.ales-corp.co.jp"
        ntripMountpoint="RTCM32M7S"
        sendGGA = True
    elif use_case_gnss_correction == "geortk":
        ntripUsername=""
        ntripPassword=""
        ntripPort=2101
        ntripUrl="geortk.jp"
        ntripMountpoint="kaname"
        sendGGA = False

class RTKParam:
    if use_case_robot == "icart3":
        dist_l_r = 0.36 # 左のアンテナから見た右のアンテナまでの距離[m]
        dist_r_c = 0.32 # 右のアンテナから見た中心のアンテナまでの距離[m]
        dist_c_l = 0.32 # 中心のアンテナから見た左のアンテナまでの距離[m]
        dist_o_rl = 0.27 # 左右のアンテナを結ぶ線と原点の距離[m]
    elif use_case_robot == "thouzer_cross":
        dist_l_r = 0.48 # 左のアンテナから見た右のアンテナまでの距離[m]
        dist_r_c = 0.45 # 右のアンテナから見た中心のアンテナまでの距離[m]
        dist_c_l = 0.45 # 中心のアンテナから見た左のアンテナまでの距離[m]
        dist_o_rl = 0.38 # 左右のアンテナを結ぶ線と原点の距離[m]
    tolerance = 0.1 # 距離チェックで許容できる誤差[m]
    antenna_ellipsoid_height = 66 # アンテナの楕円体高度 = ジオド高度 + 地表標高 + アンテナの地表高さ

class MTParam:
    map_split_m = 2 # [m]
    map_split_rad = 3.14/2 #[rad]
    map_split_thresh = 1
    min_distance_to_target = 0.8 if use_case_robot == "icart3" else 0.8 + 0.1
    deceleration_threshold_distance = 1.5 # 速度にオフセットを与え始める閾値[m]
    non_stop_offset_l = 1.0 # waypointに近づいたときに停止せずに走行するための制御指令値lのoffset[m]
    diff_threshold_m = 0.2 # waypoint通過判定用、目標と現在についての原点位置間距離と先端位置距離の和
    path_ratio_threshold_rate = 0.1 # waypoint通過判定用
    overline_threshold_m = -0.1 # waypoint通過判定用, 目標先端に引いたゴールラインに対する位置、マイナスは手前

class PlotParam:
    log_file_path = 'log20241024-151306.csv'
    log_header = ["中心アンテナ-受信時刻", "timestamp", "緯度", "経度", "高度＋ジオイド高", "品質", 
                  "右アンテナ-受信時刻", "timestamp", "緯度", "経度", "高度＋ジオイド高", "品質",
                  "左アンテナ-受信時刻", "timestamp", "緯度", "経度", "高度＋ジオイド高", "品質", 
                  "現在の車体中心の緯度", "現在の車体中心の経度", "現在の車体の方位角", 
                  "車体姿勢", "目標の緯度", "目標の経度", "目標の方位角", 
                  "目標位置と現在位置の距離", "目標位置と現在姿勢の方位角の差"]
