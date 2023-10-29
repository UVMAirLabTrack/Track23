cmake_minimum_required(VERSION 3.5)
project(led_control)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(led_control src/light_control.py)

# Add dependencies
ament_target_dependencies(led_control
  rclcpp
  std_msgs
)

# Install
install(TARGETS
  led_control
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()