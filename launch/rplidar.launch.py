import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    params = {'serial_port': '/dev/ttyUSB0', 
              'frame_id': 'laser_frame', 
              'angle_compensate': True, 
              'scan_mode': 'Standard'}
    
    node_rplidar_ros = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_rplidar_ros
    ])
