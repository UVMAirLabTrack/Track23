cmake_minimum_required(VERSION 3.8)
project(world_gen)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/data
    ${CMAKE_CURRENT_LIST_DIR}/launch
    ${CMAKE_CURRENT_LIST_DIR}/worlds
    DESTINATION share/${PROJECT_NAME}/
)
ament_package()
