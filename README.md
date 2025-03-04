# Gazebo Cafe with TurtleBot3 Burger (ROS 2 Humble)
![Cafe_with_Burger](https://github.com/910514/gazebo-cafe-with-burger/blob/main/images/demo.png)
This repository, `gazebo-cafe-with-burger`, provides a custom Gazebo environment featuring a cafe world (`cafe.world`) designed for use with the TurtleBot3 Burger robot in ROS 2 Humble (Ubuntu 22.04). It includes launch files to spawn the TurtleBot3 in the cafe environment and explore it within Gazebo.

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

2. **Update SDF Paths for Your System**:
   - The SDF uses absolute paths specific to the original developer’s system (`/home/josh`). Run this command to replace them with your home directory:
     ```bash
     sed -i "s|/home/josh|$HOME|g" ~/ros2_ws/src/cafe_env/models/turtlebot3_burger/model.sdf
     ```

3. **Build the Workspace & Setup required models**:
   Build the workspace using `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   cd ~/Downloads
   git clone https://github.com/osrf/gazebo_models.git
   cp -dr ~/Downloads/gazebo_models/cafe ~/.gazebo/models
   cp -dr ~/Downloads/gazebo_models/cafe_table ~/.gazebo/models
   ```

## Usage

### Launching the Cafe Environment with TurtleBot3

   ```bash
   ros2 launch cafe_env launch_cafe_world.launch.py
   ```