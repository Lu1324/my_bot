import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    package_name='my_bot'

    config_dir = os.path.join(get_package_share_directory(package_name),'config')

    launch_params_file = os.path.join(config_dir, 'mapper_params_online_async.yaml')
    teleop_twist_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[
                ('/cmd_vel', '/diff_cont/cmd_vel_unstamped')
            ],
        prefix='mate-terminal -- '
    )

    rivz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[os.path.join(config_dir, 'nav_rviz_config1.rviz')],
        prefix = 'mate-terminal -- ',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
        ),
        launch_arguments={
            'params_file': launch_params_file,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        slam_toolbox,
        rivz2_node,
        teleop_twist_node,
    ])