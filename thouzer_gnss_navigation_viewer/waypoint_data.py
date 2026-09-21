"""ROSに依存しないCSV読込とWGS84座標変換。"""
import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class Waypoint:
    latitude: float
    longitude: float
    yaw_deg: float
    pause: bool


def validate_origin(latitude, longitude, height):
    if not all(math.isfinite(v) for v in (latitude, longitude, height)):
        raise ValueError('Origin latitude, longitude and height must be finite')
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError('Origin latitude/longitude out of range')


def read_waypoints(filename):
    points = []
    with Path(filename).expanduser().open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'緯度', '経度', '方位', 'pause'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV header requires 緯度,経度,方位,pause')
        for row in reader:
            try:
                lat, lon, yaw = (float(row[key]) for key in ('緯度', '経度', '方位'))
                validate_origin(lat, lon, 0.0)
                if not math.isfinite(yaw):
                    raise ValueError('Heading must be finite')
                pause = row['pause'].strip().lower()
                if pause not in ('true', 'false'):
                    raise ValueError('pause must be True or False')
                points.append(Waypoint(lat, lon, yaw, pause == 'true'))
            except (ValueError, TypeError, AttributeError) as error:
                raise ValueError(f'CSV line {reader.line_num}: {error}') from error
    if not points:
        raise ValueError('CSV contains no waypoints')
    return points


def _ecef(lat, lon, height):
    b, l = math.radians(lat), math.radians(lon)
    e2 = 6.6943799901413165e-3
    n = 6378137.0 / math.sqrt(1 - e2 * math.sin(b)**2)
    return ((n + height) * math.cos(b) * math.cos(l),
            (n + height) * math.cos(b) * math.sin(l),
            (n * (1-e2) + height) * math.sin(b))


def to_enu(latitude, longitude, origin_latitude, origin_longitude, height=0.0):
    """2次元の経路として、CSVの各点は原点と同じ楕円体高と仮定する。"""
    validate_origin(origin_latitude, origin_longitude, height)
    x, y, z = (a-b for a, b in zip(
        _ecef(latitude, longitude, height),
        _ecef(origin_latitude, origin_longitude, height)))
    b, l = math.radians(origin_latitude), math.radians(origin_longitude)
    return (-math.sin(l)*x + math.cos(l)*y,
            -math.sin(b)*math.cos(l)*x - math.sin(b)*math.sin(l)*y + math.cos(b)*z)
