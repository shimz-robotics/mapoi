cmake_minimum_required(VERSION 3.5)
project(mapoi_webui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY
    web
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  mapoi_webui/mapoi_webui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
