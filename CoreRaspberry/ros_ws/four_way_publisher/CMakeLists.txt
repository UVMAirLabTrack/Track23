cmake_minimum_required(VERSION 3.5)
project(four_way_publisher)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(four_way_publisher src/four_way_pub.py)

# Add dependencies
ament_target_dependencies(
four_way_publisher
  rclcpp
  std_msgs
)

# Install
install(TARGETS
  four_way_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()
