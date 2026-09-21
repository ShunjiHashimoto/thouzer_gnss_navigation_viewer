"""固定IDを使い、静的なマーカーの全件データを生成する。"""
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from .waypoint_data import to_enu


def make_markers(waypoints, origin, frame_id='map', show_numbers=False):
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    result.markers.append(clear)

    def marker(namespace, index, kind):
        item = Marker()
        item.header.frame_id = frame_id
        # 時刻を0にして最新のTFを参照し、RVizの後起動時も古いTFを要求しない。
        item.ns, item.id, item.type = namespace, index, kind
        item.action = Marker.ADD
        item.pose.orientation.w = 1.0
        item.color.a = 1.0
        result.markers.append(item)
        return item

    line = marker('route', 0, Marker.LINE_STRIP)
    line.scale.x = 0.08
    line.color.g, line.color.b = 0.8, 1.0
    for index, waypoint in enumerate(waypoints):
        x, y = to_enu(waypoint.latitude, waypoint.longitude, *origin)
        line.points.append(Point(x=x, y=y, z=0.15))
        dot = marker('points', index, Marker.SPHERE)
        dot.pose.position = Point(x=x, y=y, z=0.3)
        dot.scale.x = dot.scale.y = dot.scale.z = 0.7 if waypoint.pause else 0.4
        dot.color.r = 1.0
        dot.color.g = 1.0 if waypoint.pause else 0.0
        arrow = marker('headings', index, Marker.ARROW)
        arrow.pose.position = Point(x=x, y=y, z=0.3)
        angle = math.radians(waypoint.yaw_deg)
        arrow.pose.orientation.z = math.sin(angle / 2)
        arrow.pose.orientation.w = math.cos(angle / 2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.9, 0.12, 0.18
        arrow.color.r, arrow.color.g = dot.color.r, dot.color.g
        if show_numbers:
            label = marker('numbers', index, Marker.TEXT_VIEW_FACING)
            label.pose.position = Point(x=x, y=y, z=1.0)
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = str(index + 1)
    return result
