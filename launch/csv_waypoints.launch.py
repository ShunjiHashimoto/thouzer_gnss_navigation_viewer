"""CSVのマーカー配信ノードだけを起動し、既存のRVizを使用する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('csv_file'),
            DeclareLaunchArgument('origin_latitude_deg'),
            DeclareLaunchArgument('origin_longitude_deg'),
            DeclareLaunchArgument('origin_ellipsoid_height_m', default_value='0.0'),
            DeclareLaunchArgument('frame_id', default_value='map'),
            DeclareLaunchArgument('show_numbers', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='false')]
    types = {'csv_file': str, 'origin_latitude_deg': float, 'origin_longitude_deg': float,
             'origin_ellipsoid_height_m': float, 'frame_id': str,
             'show_numbers': bool, 'use_sim_time': bool}
    params = {key: ParameterValue(LaunchConfiguration(key), value_type=kind)
              for key, kind in types.items()}
    return LaunchDescription(args + [Node(
        package='thouzer_gnss_navigation_viewer', executable='csv_waypoint_viewer',
        output='screen', parameters=[params])])
