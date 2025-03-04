cmake_minimum_required(VERSION 3.5)
project(cafe_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Added for Odometry
find_package(tf2 REQUIRED)       # Added for tf2::Quaternion

# Include directories
include_directories(include)

# Define dependencies
set(dependencies
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
)

# Build the turtlebot3_drive executable
add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
ament_target_dependencies(turtlebot3_drive ${dependencies})

# Install targets and directories
install(TARGETS turtlebot3_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models meshs worlds include
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_package()