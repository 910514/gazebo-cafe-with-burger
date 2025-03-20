import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    world_path = os.path.join(get_package_share_directory('cafe_env'), 'worlds', 'cafe.world')
    turtlebot_sdf_path = os.path.join(get_package_share_directory('cafe_env'), 'models', 'turtlebot3_burger', 'model.sdf')

    # Get the URDF file for TurtleBot3
    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    turtlebot3_urdf_path = os.path.join(turtlebot3_description_pkg, 'urdf', 'turtlebot3_burger.urdf')

    # Launch configuration variables for TurtleBot3
    turtlebot_x_pose = LaunchConfiguration('turtlebot_x_pose', default='0.0')
    turtlebot_y_pose = LaunchConfiguration('turtlebot_y_pose', default='0.0')
    turtlebot_z_pose = LaunchConfiguration('turtlebot_z_pose', default='0.01')
    turtlebot_roll = LaunchConfiguration('turtlebot_roll', default='0.0')
    turtlebot_pitch = LaunchConfiguration('turtlebot_pitch', default='0.0')
    turtlebot_yaw = LaunchConfiguration('turtlebot_yaw', default='0.0')

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

    # Set TURTLEBOT3_MODEL environment variable
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

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
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Robot State Publisher for TurtleBot3
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[turtlebot3_urdf_path]
    )

    # Static transform publisher for TurtleBot3 imu_link
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.032', '0', '0.068', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static transform publisher for TurtleBot3 base_scan
    static_transform_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.032', '0', '0.171', '0', '0', '0', 'base_link', 'base_scan'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(set_env)
    ld.add_action(declare_turtlebot_x_position_cmd)
    ld.add_action(declare_turtlebot_y_position_cmd)
    ld.add_action(declare_turtlebot_z_position_cmd)
    ld.add_action(declare_turtlebot_roll_cmd)
    ld.add_action(declare_turtlebot_pitch_cmd)
    ld.add_action(declare_turtlebot_yaw_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_turtlebot_spawner_cmd)
    ld.add_action(robot_state_publisher_cmd)  # Add robot_state_publisher
    ld.add_action(static_transform_imu)
    ld.add_action(static_transform_lidar)

    return ld