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

ament_auto_add_executable(mapoi_switch_map_client
  src/mapoi_switch_map_client.cpp
)

ament_auto_add_executable(navigate_to_pose_client
  src/navigate_to_pose_client.cpp
)

ament_auto_add_executable(follow_waypoints_client
  src/follow_waypoints_client.cpp
)

# PoiEvent 駆動 sample subscriber (#88)。route 走行中に audio_info / capture_trigger
# tag 付き POI で audio guide / camera trigger を mock 実行する参照実装。
ament_auto_add_executable(audio_guide_node
  src/audio_guide_node.cpp
)

ament_auto_add_executable(camera_node
  src/camera_node.cpp
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
