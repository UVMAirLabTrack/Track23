cmake_minimum_required(VERSION 3.8)
project(direct_io)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    config
    data
    launch
    DESTINATION share/${PROJECT_NAME}/)

ament_package()