import json
import math
import random
# ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
# custom
from gnss_module.coordinate import blh, getXY
import gnss_module.WGS84 as datum 
from .mqtt_handler import MqttHandler
from .config import MQTTParam

class GNSSNavigationViewer(Node):
    def __init__(self):
        super().__init__('gnss_navigation_viewer')
        # PoseStampedメッセージをpublishするパブリッシャーを作成
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'robot_pose', 10)
        # MarkerArrayメッセージをpublishするパブリッシャーを作成
        self.marker_array_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # MQTTハンドラーを作成し、メッセージ受信時のコールバックを設定
        self.mqtt_handler = MqttHandler(on_message_callback=self.handle_mqtt_message, topic_sub=MQTTParam.topic_event)
        self.mqtt_handler.start_whisperer()  # MQTTのループを開始
        # マップ初期位置を設定
        self.initial_blh = blh(datum, 36.083208948105906,140.07766251434052, 0)
    
    # MQTTメッセージを受け取った際に呼び出されるコールバック関数
    def handle_mqtt_message(self, payload: str):
        try:
            data = json.loads(payload)
            waypoint_flag = False
            if data["data"]["status"] == "waypoint" or data["data"]["status"] == "paused": waypoint_flag = True
            # GNSSデータから位置と向きを設定
            lat_deg = data["data"]["LatLonYaw"]["lat_deg"]
            lon_deg = data["data"]["LatLonYaw"]["lon_deg"]
            yaw_deg = data["data"]["LatLonYaw"]["yaw_deg"]
            # PoseStampedメッセージを作成
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            # 緯度経度をXY座標に変換
            x, y = getXY(self.initial_blh, lat_deg, lon_deg, 0.0)
            # 位置を設定
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            # Yaw角をクオータニオンに変換して方向を設定
            pose_msg.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
            pose_msg.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
            # トピックにPoseメッセージをpublish
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Published Pose: {pose_msg}")
            # MarkerArrayメッセージを作成
            marker_array_msg = MarkerArray()
            antennas = [
                {"name": "中心座標", "lat": lat_deg, "lon": lon_deg},
            ]
            for idx, antenna in enumerate(antennas):
                marker_msg = Marker()
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.header.frame_id = 'map'
                marker_msg.ns = 'gnss_navigation'
                marker_msg.id = random.randint(1, 100000)
                marker_msg.type = Marker.SPHERE
                marker_msg.action = Marker.ADD
                # 緯度経度をXY座標に変換
                x, y = getXY(self.initial_blh, antenna["lat"], antenna["lon"], 0.0)
                marker_msg.pose.position.x = x
                marker_msg.pose.position.y = y
                marker_msg.pose.position.z = 0.5
                marker_msg.pose.orientation.w = 1.0
                marker_msg.scale.x = 0.5
                marker_msg.scale.y = 0.5
                marker_msg.scale.z = 0.5
                marker_msg.color.a = 1.0
                marker_msg.color.r = 0.0
                marker_msg.color.g = 1.0
                marker_msg.color.b = 0.0
                yaw_deg = data["data"]["LatLonYaw"]["yaw_deg"]
                if data["data"]["status"] == "paused":
                    marker_msg.color.r = 1.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 0.0
                if data["data"]["status"] == "waypoint":
                    marker_msg.color.r = 1.0
                    marker_msg.color.g = 0.0
                    marker_msg.color.b = 0.0
                if waypoint_flag:
                    marker_msg.scale.x = 1.0
                    marker_msg.scale.y = 1.0
                    marker_msg.scale.z = 1.0
                marker_msg.lifetime = Duration()
                # MarkerをMarkerArrayに追加
                marker_array_msg.markers.append(marker_msg)
            # トピックにMarkerArrayメッセージをpublish
            self.marker_array_publisher_.publish(marker_array_msg)
            self.get_logger().info(f"Published MarkerArray with {len(marker_array_msg.markers)} markers")
            if waypoint_flag: return

            # ロボットのSTLファイルを表示するMarkerを追加
            robot_marker = Marker()
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.header.frame_id = 'map'
            robot_marker.ns = 'gnss_navigation'
            robot_marker.id = random.randint(1, 100000)
            robot_marker.type = Marker.MESH_RESOURCE
            robot_marker.action = Marker.ADD
            # STLファイルのパスを設定
            robot_marker.mesh_resource = "package://thouzer_gnss_navigation_viewer/meshes/RMS-SZE2.STL"
            robot_marker.mesh_use_embedded_materials = True # meshファイルの内部のcolor情報を描画
            # 中心位置はロボットの車体中心
            x, y = getXY(self.initial_blh, lat_deg, lon_deg, 0.0)
            robot_marker.pose.position.x = x 
            robot_marker.pose.position.y = y
            robot_marker.pose.position.z = 0.0
            robot_marker.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
            robot_marker.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
            robot_marker.scale.x = 0.002
            robot_marker.scale.y = 0.002
            robot_marker.scale.z = 0.002
            robot_marker.color.a = 0.5
            robot_marker.color.r = 0.5
            robot_marker.color.g = 0.5
            robot_marker.color.b = 0.5
            robot_marker.lifetime = Duration()
            robot_marker.lifetime.sec = 0
            robot_marker.lifetime.nanosec = 200000000
            # MarkerArrayに追加
            marker_array_msg.markers.append(robot_marker)
            # トピックにロボットSTLマーカーをpublish
            self.marker_array_publisher_.publish(marker_array_msg)
            self.get_logger().info(f"Published Robot STL Marker")

        except Exception as e:
            self.get_logger().error(f"Failed to process MQTT message: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = GNSSNavigationViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()