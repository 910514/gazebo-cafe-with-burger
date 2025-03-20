# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    world_path = os.path.join(get_package_share_directory('cafe_env'), 'worlds', 'cafe.world')
<<<<<<< Updated upstream
    sdf_path = os.path.join(get_package_share_directory('cafe_env'), 'models', 'turtlebot3_burger', 'model.sdf')

    # Launch configuration variables with intended defaults (position and orientation)
    x_pose = LaunchConfiguration('x_pose', default='-0.776521328089704')
    y_pose = LaunchConfiguration('y_pose', default='-1.5538895640656596')
    z_pose = LaunchConfiguration('z_pose', default='0.1983192687124241')
    roll = LaunchConfiguration('roll', default='0.0')      # From quaternion, negligible
    pitch = LaunchConfiguration('pitch', default='0.0')    # From quaternion, negligible
    yaw = LaunchConfiguration('yaw', default='1.576')      # Approx 90 degrees from quaternion

    # Declare the launch arguments with matching defaults
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='-0.776521328089704',
        description='X position of the robot in the world'
    )
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-1.5538895640656596',
        description='Y position of the robot in the world'
    )
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.1983192687124241',
        description='Z position of the robot in the world'
    )
    declare_roll_cmd = DeclareLaunchArgument(
        'roll', default_value='0.0',
        description='Roll orientation of the robot in radians'
    )
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch', default_value='0.0',
        description='Pitch orientation of the robot in radians'
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='1.576',
        description='Yaw orientation of the robot in radians'
    )

    # Start Gazebo server with ROS plugins
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', world_path, '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
=======
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
>>>>>>> Stashed changes
        output='screen'
    )

    # Start Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

<<<<<<< Updated upstream
    # Spawn the TurtleBot3 model with position and orientation
    spawn_model = Node(
=======
    # Spawn TurtleBot3 robot
    start_turtlebot_spawner_cmd = Node(
>>>>>>> Stashed changes
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,    # Roll in radians
            '-P', pitch,   # Pitch in radians
            '-Y', yaw      # Yaw in radians
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

    # Create the launch description
    ld = LaunchDescription()
<<<<<<< Updated upstream
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_model)
=======
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
>>>>>>> Stashed changes

    return ld