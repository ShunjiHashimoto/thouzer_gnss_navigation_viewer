from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('thouzer_gnss_navigation_viewer')
    default_rviz_config_path = PathJoinSubstitution([pkg_path, 'rviz', 'gnss.rviz'])

    return LaunchDescription([
        # RVizの設定ファイルのパスを引数として受け取る
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Path to the RVIZ config file'
        ),
        Node(
            package='thouzer_gnss_navigation_viewer',
            executable='gnss_viewer',
            name='gnss_navigation_viewer',
            output='screen'
        ),
        # RVizノードを起動
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        )
    ])