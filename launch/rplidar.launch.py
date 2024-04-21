import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    # Create a Node
    # ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard
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


    # Launch!
    return LaunchDescription([
        node_rplidar_ros
    ])
