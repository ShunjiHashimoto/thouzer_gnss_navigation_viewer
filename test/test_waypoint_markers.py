"""ROS 2の環境設定を読み込んで実行するメッセージ生成テスト。"""
import math
import pytest
pytest.importorskip('visualization_msgs.msg')
from visualization_msgs.msg import Marker
from thouzer_gnss_navigation_viewer.waypoint_data import Waypoint
from thouzer_gnss_navigation_viewer.waypoint_markers import make_markers


def test_snapshot_and_heading():
    points = [Waypoint(36, 140, -90, True), Waypoint(36, 140.00001, 0, False)]
    result = make_markers(points, (36, 140, 0), show_numbers=True)
    assert result.markers[0].action == Marker.DELETEALL
    markers = {(m.ns,m.id): m for m in result.markers[1:]}
    assert len(markers) == 7
    assert markers['points',0].color.g == 1
    assert markers['points',1].color.g == 0
    assert markers['numbers',0].text == '1'
    assert markers['headings',0].pose.orientation.z == pytest.approx(-math.sqrt(.5))
    assert all(m.header.frame_id == 'map' for m in result.markers[1:])
    assert [(m.ns,m.id) for m in result.markers] == [(m.ns,m.id) for m in make_markers(points,(36,140,0),show_numbers=True).markers]
