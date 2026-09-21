"""ROSに依存しないCSV読込と測地座標変換のテスト。"""
import csv
import importlib.util
import math
from pathlib import Path
import pytest
from thouzer_gnss_navigation_viewer.waypoint_data import read_waypoints, to_enu, validate_origin


@pytest.mark.parametrize('row', [
    'nan,140,0,False', '91,140,0,False', '36,181,0,False',
    '36,140,inf,False', '36,140,0,maybe', '36,140,0,', '36,140',
])
def test_bad_row_reports_line(tmp_path, row):
    file = tmp_path/'bad.csv'
    file.write_text('緯度,経度,方位,pause\n'+row+'\n', encoding='utf-8')
    with pytest.raises(ValueError, match='line 2'):
        read_waypoints(file)


def test_bom_and_pause(tmp_path):
    file = tmp_path/'valid.csv'
    file.write_text('緯度,経度,方位,pause\r\n36,140,-90,True\r\n36,140,0,False\r\n', encoding='utf-8-sig')
    points = read_waypoints(file)
    assert [p.pause for p in points] == [True, False]
    assert points[0].yaw_deg == -90


@pytest.mark.parametrize('contents', ['', 'lat,lon\n1,2\n', '緯度,経度,方位,pause\n'])
def test_empty_or_invalid_header(tmp_path, contents):
    file = tmp_path/'bad.csv'
    file.write_text(contents)
    with pytest.raises(ValueError):
        read_waypoints(file)


def test_missing_and_invalid_origin(tmp_path):
    with pytest.raises(FileNotFoundError):
        read_waypoints(tmp_path/'missing.csv')
    with pytest.raises(ValueError):
        validate_origin(float('nan'), 140, 0)
    assert to_enu(36, 140, 36, 140, 100) == (0, 0)
    east, north = to_enu(36, 140.00001, 36, 140)
    assert 0.8 < east < 1.0 and abs(north) < 0.001
    east, north = to_enu(36.00001, 140, 36, 140)
    assert abs(east) < 0.001 and 1.0 < north < 1.2


def test_site_101_against_map_generator():
    workspace = Path(__file__).resolve().parents[2]
    generator = workspace/'dne_turtlebot_flatland/maps/tsukuba_cityhall/generate.py'
    csv_file = Path.home()/'git/GnssNav/csv/101.csv'
    if not generator.exists() or not csv_file.exists():
        pytest.skip('Site data is not installed in this checkout')
    spec = importlib.util.spec_from_file_location('site_map_generator', generator)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    points = read_waypoints(csv_file)
    assert len(points) == 83
    assert [i for i,p in enumerate(points) if p.pause] == [0, 39]
    for point in points:
        size = 256 * 2**module.G['tile_zoom']
        pixel = [(point.longitude+180)/360*size-module.G['crop_global_pixel'][0],
                 (1-math.asinh(math.tan(math.radians(point.latitude)))/math.pi)/2*size-module.G['crop_global_pixel'][1]]
        expected = module.enu(pixel)
        actual = to_enu(point.latitude, point.longitude, module.LAT, module.LON)
        assert math.dist(expected, actual) < 0.001
    first = to_enu(points[0].latitude, points[0].longitude, module.LAT, module.LON)
    assert first == pytest.approx((90.584, -31.740), abs=0.001)
    assert math.radians(points[0].yaw_deg) == pytest.approx(-0.15571930954810687)
