# Gazebo Cafe with TurtleBot3 Burger (ROS 2 Humble)
![Cafe_with_Burger](https://github.com/910514/gazebo-cafe-with-burger/blob/main/images/demo.png)
This repository, `gazebo-cafe-with-burger`, provides a custom Gazebo environment featuring a cafe world (`cafe.world`) designed for use with the TurtleBot3 Burger robot in ROS 2 Humble (Ubuntu 22.04). It includes launch files to spawn the TurtleBot3 in the cafe environment and explore it within Gazebo.

## Prerequisites

- **ROS 2 Humble**: Installed on Ubuntu 22.04.
- **Gazebo**: ROS 2 Humble typically uses Gazebo Classic (version 11). Install it with:
- **TurtleBot3 Packages**: Install the ROS 2 TurtleBot3 simulation packages:
- **Colcon**: Build tool for ROS 2.

## Installation

1. **Clone the Repository**:
   Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/910514/gazebo-cafe-with-burger.git
   mv ~/ros2_ws/src/gazebo-cafe-with-burger/cafe_env ~/ros2_ws/src/
   sudo rm -dr ~/ros2_ws/src/gazebo-cafe-with-burger
   ```

2. **Build the Workspace**:
   Build the workspace using `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   mkdir ~/ros2_ws/src/install/cafe_env/share/cafe_env/worlds
   mkdir ~/ros2_ws/src/install/cafe_env/share/cafe_env/launch
   cp ~/ros2_ws/src/cafe_env/worlds/cafe.world ~/ros2_ws/src/install/cafe_env/share/cafe_env/worlds
   cp ~/ros2_ws/src/cafe_env/launch/robot_state_publisher.launch.py ~/ros2_ws/src/install/cafe_env/share/cafe_env/launch
   cp ~/ros2_ws/src/cafe_env/launch/spawn_turtlebot3.launch.py  ~/ros2_ws/src/install/cafe_env/share/cafe_env/launch
   cp ~/ros2_ws/src/cafe_env/launch/cafe_env.launch.py ~/ros2_ws/src/install/cafe_env/share/cafe_env
   ```

3. **Set TurtleBot3 Model**:
   Set the environment variable for the TurtleBot3 model to "burger":
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

## Folder Structure

```
gazebo-cafe-with-burger/
├── cafe_env/               # Python package for environment-specific scripts
│   └── __init__.py
├── launch/                 # Launch files for starting the simulation
│   ├── cafe_env.launch.py  # Launches the cafe world in Gazebo
│   ├── robot_state_publisher.launch.py  # Publishes robot state
│   └── spawn_turtlebot3.launch.py       # Spawns TurtleBot3 in the cafe
├── package.xml             # ROS 2 package manifest
├── resource/               # Resource folder (e.g., for RViz configs or meshes)
│   └── cafe_env
├── setup.cfg              # Configuration for Python setup
├── setup.py               # Python package setup script
├── test/                   # Test scripts for code quality
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── worlds/                 # Gazebo world files
    └── cafe.world          # Custom cafe environment
```

## Usage

### Launching the Cafe Environment with TurtleBot3

   ```bash
   ros2 launch cafe_env cafe_env.launch.py
   ```

## Building and Running

1. After editing files, rebuild:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select cafe_env
   source install/setup.bash
   ```
2. Run the launch files as shown above.