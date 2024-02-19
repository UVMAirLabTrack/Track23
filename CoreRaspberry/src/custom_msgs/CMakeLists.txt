cmake_minimum_required(VERSION 3.8)
project(custom_msgs)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})
#find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/world_marker_msg.msg"
  #"msg/Sphere.msg"
  #"srv/AddThreeInts.srv"
  #DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


install(DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/data
    ${CMAKE_CURRENT_LIST_DIR}/launch
    ${CMAKE_CURRENT_LIST_DIR}/worlds
    DESTINATION share/${PROJECT_NAME}/
)
ament_package()
