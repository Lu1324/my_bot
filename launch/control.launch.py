import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit

# rviz2
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=false

def generate_launch_description():
    # Get the share directory of each package

    package_name='my_bot'

    config_dir = os.path.join(get_package_share_directory(package_name),'config')

    launch_params_file = os.path.join(config_dir, 'mapper_params_online_async.yaml')
    # Start teleop_twist_keyboard in a new terminal
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

    # Start RViz
    rivz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[os.path.join(config_dir, 'nav_rviz_config1.rviz')],
        prefix = 'mate-terminal -- ',
    )


    
    # Include navigation toolbox (Nav2) launch file
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
        ),
        launch_arguments={
            'params_file': launch_params_file,
            'use_sim_time': 'false'
        }.items()
    )
    # delayed_rviz = RegisterEventHandler(
    #     event_handler = OnProcessStart(
    #         target_action=slam_toolbox,
    #         on_start=[rivz2_node],
    #     )
    # )
    return LaunchDescription([
        slam_toolbox,
        rivz2_node,
        teleop_twist_node,
    ])