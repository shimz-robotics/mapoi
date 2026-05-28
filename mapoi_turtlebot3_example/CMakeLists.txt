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

# PoiEvent 駆動 demo subscriber (#88 / #238)。route 走行中の特定 POI でアクションを
# mock 実行する参照実装。集約 launch から外して mapoi_event_samples.launch.yaml
# 単独で起動する分離方針 (#238)。
#   - audio_guide_node: EVENT_ENTER + `audio_info` tag → 音声ガイド再生 mock
#   - camera_node:      EVENT_PAUSED + `capture_trigger` tag → 撮影 mock + auto-resume
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # camera_node の挙動を pin する unit test (#248)。
  # UNIT_TEST マクロで camera_node.cpp の main() を抜き、テスト fixture を
  # friend に許可した状態で再 compile する (mapoi_server の test_nav2_bridge_unit
  # と同じパターン)。
  ament_auto_add_gtest(test_camera_node
    test/test_camera_node.cpp
    src/camera_node.cpp
  )
  target_compile_definitions(test_camera_node PRIVATE UNIT_TEST)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
