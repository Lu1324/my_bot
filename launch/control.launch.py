import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# rviz2
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=false

def generate_launch_description():
    # Get the share directory of each package
    nav2_launch_dir = get_package_share_directory('nav2_bringup') + '/launch'    

    # Start teleop_twist_keyboard in a new terminal
    Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{
            'scale_linear': 0.5,
            'scale_angular': 1.0,
        }],
        prefix='gnome-terminal -- '
    ),

    # Start RViz
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir + '/your_rviz_config.rviz']
    ),

    # Include navigation toolbox (Nav2) launch file
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={'use_sim_time': 'false'}.items()
    ),
    
    return LaunchDescription([

    ])