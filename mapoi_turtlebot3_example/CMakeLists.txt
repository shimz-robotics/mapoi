cmake_minimum_required(VERSION 3.8)
project(mapoi_turtlebot3_example)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(get_maps_info_client
  src/get_maps_info_client.cpp
)

ament_auto_add_executable(get_pois_info_client
  src/get_pois_info_client.cpp
)

ament_auto_add_executable(print_initialpose
  src/print_initialpose.cpp
)

ament_auto_add_executable(switch_map_client
  src/switch_map_client.cpp
)

ament_auto_add_executable(navigate_to_pose_client
  src/navigate_to_pose_client.cpp
)

ament_auto_add_executable(follow_waypoints_client
  src/follow_waypoints_client.cpp
)


install(
  DIRECTORY
    launch
    maps
    param
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
