import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    world_path = os.path.join(get_package_share_directory('cafe_env'), 'worlds', 'cafe.world')
    wheeltec_sdf_path = os.path.join(get_package_share_directory('wheeltec_description'), 'models', 'wheeltec', 'wheeltec.sdf')
    turtlebot_sdf_path = os.path.join(get_package_share_directory('cafe_env'), 'models', 'turtlebot3_burger', 'model.sdf')

    # Launch configuration variables for Wheeltec
    wheeltec_x_pose = LaunchConfiguration('wheeltec_x_pose', default='-0.776521328089704')
    wheeltec_y_pose = LaunchConfiguration('wheeltec_y_pose', default='-1.5538895640656596')
    wheeltec_z_pose = LaunchConfiguration('wheeltec_z_pose', default='0.1983192687124241')
    wheeltec_roll = LaunchConfiguration('wheeltec_roll', default='0.0')
    wheeltec_pitch = LaunchConfiguration('wheeltec_pitch', default='0.0')
    wheeltec_yaw = LaunchConfiguration('wheeltec_yaw', default='1.576')

    # Launch configuration variables for TurtleBot3
    turtlebot_x_pose = LaunchConfiguration('turtlebot_x_pose', default='0.0')
    turtlebot_y_pose = LaunchConfiguration('turtlebot_y_pose', default='0.0')
    turtlebot_z_pose = LaunchConfiguration('turtlebot_z_pose', default='0.01')
    turtlebot_roll = LaunchConfiguration('turtlebot_roll', default='0.0')
    turtlebot_pitch = LaunchConfiguration('turtlebot_pitch', default='0.0')
    turtlebot_yaw = LaunchConfiguration('turtlebot_yaw', default='0.0')

    # Declare launch arguments for Wheeltec
    declare_wheeltec_x_position_cmd = DeclareLaunchArgument(
        'wheeltec_x_pose', default_value='-1.1974736651732434',
        description='X position of the Wheeltec robot in the world'
    )
    declare_wheeltec_y_position_cmd = DeclareLaunchArgument(
        'wheeltec_y_pose', default_value='-1.5625352204131713',
        description='Y position of the Wheeltec robot in the world'
    )
    declare_wheeltec_z_position_cmd = DeclareLaunchArgument(
        'wheeltec_z_pose', default_value='0.24759061187997547',
        description='Z position of the Wheeltec robot in the world'
    )
    declare_wheeltec_roll_cmd = DeclareLaunchArgument(
        'wheeltec_roll', default_value='-0.00023287248797829502',
        description='Roll orientation of the Wheeltec robot in radians'
    )
    declare_wheeltec_pitch_cmd = DeclareLaunchArgument(
        'wheeltec_pitch', default_value='0.00023697728803076363',
        description='Pitch orientation of the Wheeltec robot in radians'
    )
    declare_wheeltec_yaw_cmd = DeclareLaunchArgument(
        'wheeltec_yaw', default_value='1.54579',
        description='Yaw orientation of the Wheeltec robot in radians'
    )

    # Declare launch arguments for TurtleBot3
    declare_turtlebot_x_position_cmd = DeclareLaunchArgument(
        'turtlebot_x_pose', default_value='-0.6986080477717773',
        description='X position of the TurtleBot3 robot in the world'
    )
    declare_turtlebot_y_position_cmd = DeclareLaunchArgument(
        'turtlebot_y_pose', default_value='-1.6218021524181305',
        description='Y position of the TurtleBot3 robot in the world'
    )
    declare_turtlebot_z_position_cmd = DeclareLaunchArgument(
        'turtlebot_z_pose', default_value='0.19832978849892852',
        description='Z position of the TurtleBot3 robot in the world'
    )
    declare_turtlebot_roll_cmd = DeclareLaunchArgument(
        'turtlebot_roll', default_value='0.0022079014788342535',
        description='Roll orientation of the TurtleBot3 robot in radians'
    )
    declare_turtlebot_pitch_cmd = DeclareLaunchArgument(
        'turtlebot_pitch', default_value='0.0018998916895399695',
        description='Pitch orientation of the TurtleBot3 robot in radians'
    )
    declare_turtlebot_yaw_cmd = DeclareLaunchArgument(
        'turtlebot_yaw', default_value='-1.54579',
        description='Yaw orientation of the TurtleBot3 robot in radians'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', world_path, '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Spawn Wheeltec robot
    start_wheeltec_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wheeltec',
            '-file', wheeltec_sdf_path,
            '-x', wheeltec_x_pose,
            '-y', wheeltec_y_pose,
            '-z', wheeltec_z_pose,
            '-R', wheeltec_roll,
            '-P', wheeltec_pitch,
            '-Y', wheeltec_yaw
        ],
        output='screen'
    )

    # Spawn TurtleBot3 robot
    start_turtlebot_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', turtlebot_sdf_path,
            '-x', turtlebot_x_pose,
            '-y', turtlebot_y_pose,
            '-z', turtlebot_z_pose,
            '-R', turtlebot_roll,
            '-P', turtlebot_pitch,
            '-Y', turtlebot_yaw
        ],
        output='screen'
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_wheeltec_x_position_cmd)
    ld.add_action(declare_wheeltec_y_position_cmd)
    ld.add_action(declare_wheeltec_z_position_cmd)
    ld.add_action(declare_wheeltec_roll_cmd)
    ld.add_action(declare_wheeltec_pitch_cmd)
    ld.add_action(declare_wheeltec_yaw_cmd)
    ld.add_action(declare_turtlebot_x_position_cmd)
    ld.add_action(declare_turtlebot_y_position_cmd)
    ld.add_action(declare_turtlebot_z_position_cmd)
    ld.add_action(declare_turtlebot_roll_cmd)
    ld.add_action(declare_turtlebot_pitch_cmd)
    ld.add_action(declare_turtlebot_yaw_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_wheeltec_spawner_cmd)
    ld.add_action(start_turtlebot_spawner_cmd)

    return ld