cmake_minimum_required(VERSION 3.5)
project(mapoi_server)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mapoi_poi_server
  src/mapoi_poi_server.cpp
)

ament_auto_add_executable(mapoi_nav_server
  src/mapoi_nav_server.cpp
)

ament_auto_add_executable(mapoi_rviz2_publisher
  src/mapoi_rviz2_publisher.cpp
)


install(
  DIRECTORY
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
