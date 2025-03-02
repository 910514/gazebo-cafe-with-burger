import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    cafe_env_pkg = get_package_share_directory('cafe_env')

    # Path to your custom world file
    world_file = os.path.join(cafe_env_pkg, 'worlds', 'cafe.world')

    # Launch Gazebo with your custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Include the robot state publisher launch file
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cafe_env_pkg, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    # Include the spawn TurtleBot3 launch file
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cafe_env_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        )
    )

    # Gazebo client (optional, for visualization)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            )
        )
    )

    return LaunchDescription([
        gazebo,
        gazebo_client,  # Optional: Remove if you don’t need the GUI
        robot_state_publisher,
        spawn_turtlebot,
    ])