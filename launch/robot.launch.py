import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():

    package_name='my_bot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control= LaunchConfiguration('use_ros2_control')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=true', ' sim_mode:=false'])

    # Create a robot_state_publisher node
    robot_description = {'robot_description': robot_description_config} 
    

    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package='controller_manager', 
        executable='ros2_control_node',  
        parameters=[controller_params_file],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': False}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    joint_broad_spawner = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['joint_broad', "--controller-manager", "/controller_manager"])
    


    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action = controller_manager,
            on_start = [joint_broad_spawner],
        )
    )
    diff_drive_spawner = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['diff_cont', "--controller-manager", "/controller_manager"])

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )
    
    node_rplidar_ros = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB0', 
            'frame_id': 'laser_frame', 
            'angle_compensate': True, 
            'scan_mode': 'Standard'}]
    )

    delayed_rplidar = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[node_rplidar_ros],
        )
    )
    
    return LaunchDescription([
        controller_manager,
        node_robot_state_publisher,
        joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_rplidar,
    ])
