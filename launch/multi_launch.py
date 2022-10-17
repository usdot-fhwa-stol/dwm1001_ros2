import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions as lra
from launch_ros.actions import Node

def generate_launch_description():
    multi_ips_config = os.path.join(
        get_package_share_directory('dwm1001_ros2'),
        'config',
        'multi_ips.yaml'
    )

    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node'
        ),
        Node(
            package='laser_converter',
            executable='laser_to_pcd2'
        ),
        Node(
            package='dwm1001_ros2',
            executable='dwm1001_heading_corrector'
        ),
        Node(
            package='dwm1001_ros2',
            executable='dwm1001',
            name='front_dwm1001',
            parameters=[multi_ips_config]
        ),
        Node(
            package='dwm1001_ros2',
            executable='dwm1001',
            name='rear_dwm1001',
            parameters=[multi_ips_config]
        )
    ])