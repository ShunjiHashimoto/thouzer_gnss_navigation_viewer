"""MQTTやロボットモデルに依存せず、CSVの経路を表示する。"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from .waypoint_data import read_waypoints, validate_origin
from .waypoint_markers import make_markers


class CSVWaypointViewer(Node):
    def __init__(self):
        super().__init__('csv_waypoint_viewer')
        try:
            self.declare_parameter('csv_file', '')
            self.declare_parameter('origin_latitude_deg', Parameter.Type.DOUBLE)
            self.declare_parameter('origin_longitude_deg', Parameter.Type.DOUBLE)
            self.declare_parameter('origin_ellipsoid_height_m', 0.0)
            self.declare_parameter('frame_id', 'map')
            self.declare_parameter('show_numbers', False)
            origin = tuple(self.get_parameter(key).value for key in (
                'origin_latitude_deg', 'origin_longitude_deg', 'origin_ellipsoid_height_m'))
            validate_origin(*origin)
            filename = self.get_parameter('csv_file').value
            if not filename:
                raise ValueError('csv_file is required')
            frame = self.get_parameter('frame_id').value
            if not frame.strip():
                raise ValueError('frame_id must not be empty')
            points = read_waypoints(filename)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(MarkerArray, '/gnss/waypoint_markers', qos)
            self.publisher.publish(make_markers(
                points, origin, frame, self.get_parameter('show_numbers').value))
            self.get_logger().info(f'Published {len(points)} waypoints from {filename} in {frame}')
        except Exception:
            self.destroy_node()
            raise


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CSVWaypointViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
