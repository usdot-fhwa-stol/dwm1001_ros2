import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    multi_ips_config = os.path.join(
        get_package_share_directory('dwm1001_ros2'),
        'config',
        'multi_ips.yaml'
    )
    rear_ips_config = os.path.join(
        get_package_share_directory('dwm1001_ros2'),
        'config',
        'rear_ips.yaml'
    )

    return LaunchDescription([
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