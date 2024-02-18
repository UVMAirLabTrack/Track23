cmake_minimum_required(VERSION 3.8)
project(rqt_plugs)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

add_executable(four_way_rqt src/four_way_rqt.py)
ament_target_dependencies(four_way_rqt rclpy std_msgs)

install(TARGETS four_way_rqt
        DESTINATION lib/${PROJECT_NAME})

ament_package()