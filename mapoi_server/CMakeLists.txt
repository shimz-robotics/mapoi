cmake_minimum_required(VERSION 3.5)
project(mapoi_server)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# yaml-cpp: ament_auto does not reliably propagate the yaml-cpp link library
# on Jazzy (yaml-cpp 0.8 exports yaml-cpp::yaml-cpp, Humble's 0.7 uses yaml-cpp).
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(_mapoi_yaml_cpp_target yaml-cpp::yaml-cpp)
else()
  set(_mapoi_yaml_cpp_target yaml-cpp)
endif()

ament_auto_add_executable(mapoi_server
  src/mapoi_server.cpp
  src/initial_pose_resolver.cpp
)
target_link_libraries(mapoi_server ${_mapoi_yaml_cpp_target})

ament_auto_add_executable(mapoi_nav2_bridge
  src/mapoi_nav2_bridge.cpp
)

ament_auto_add_executable(mapoi_amcl_localization_bridge
  src/mapoi_amcl_localization_bridge.cpp
)

ament_auto_add_executable(mapoi_rviz2_publisher
  src/mapoi_rviz2_publisher.cpp
)
target_link_libraries(mapoi_rviz2_publisher ${_mapoi_yaml_cpp_target})

# mapoi_gazebo_bridge: Gazebo Classic (gazebo_msgs) 用、Humble で build される。
# Jazzy では gazebo_msgs が存在しないため find_package QUIET で skip。
find_package(gazebo_msgs QUIET)
if(gazebo_msgs_FOUND)
  ament_auto_add_executable(mapoi_gazebo_bridge
    src/mapoi_gazebo_bridge.cpp
    src/initial_pose_resolver.cpp
  )
  target_link_libraries(mapoi_gazebo_bridge ${_mapoi_yaml_cpp_target})
  ament_target_dependencies(mapoi_gazebo_bridge gazebo_msgs)
  message(STATUS "Building mapoi_gazebo_bridge (Gazebo Classic)")
else()
  message(STATUS "Skipping mapoi_gazebo_bridge (gazebo_msgs not found)")
endif()

# mapoi_gz_bridge: gz-sim (ros_gz_interfaces) 用、Jazzy 以降で build される。
# Humble の image にも ros_gz_interfaces が含まれている場合があるが、
# Humble の rclcpp は create_client(name, rclcpp::QoS, group) シグネチャを持たないため、
# find_package が成功しても build は skip する必要がある。
# package.xml の condition と揃え、ROS_DISTRO != humble のときのみ build する。
find_package(ros_gz_interfaces QUIET)
if(ros_gz_interfaces_FOUND AND NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  ament_auto_add_executable(mapoi_gz_bridge
    src/mapoi_gz_bridge.cpp
    src/initial_pose_resolver.cpp
  )
  target_link_libraries(mapoi_gz_bridge ${_mapoi_yaml_cpp_target})
  ament_target_dependencies(mapoi_gz_bridge ros_gz_interfaces)
  message(STATUS "Building mapoi_gz_bridge (gz-sim, ROS_DISTRO=$ENV{ROS_DISTRO})")
else()
  message(STATUS "Skipping mapoi_gz_bridge "
    "(ros_gz_interfaces_FOUND=${ros_gz_interfaces_FOUND}, ROS_DISTRO=$ENV{ROS_DISTRO})")
endif()


install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Level 1: gtest unit tests
  ament_auto_add_gtest(test_nav2_bridge_unit
    test/test_nav2_bridge_unit.cpp
    src/mapoi_nav2_bridge.cpp
  )
  target_compile_definitions(test_nav2_bridge_unit PRIVATE UNIT_TEST)

  # initial_pose_resolver は純関数なので test には resolver の cpp だけを link する
  # (#150 で mapoi_server.cpp 全体 link から軽量化、Node 依存を test から排除)。
  ament_auto_add_gtest(test_initial_pose_resolver
    test/test_initial_pose_resolver.cpp
    src/initial_pose_resolver.cpp
  )
  target_compile_definitions(test_initial_pose_resolver PRIVATE UNIT_TEST)
  # initial_pose_resolver.cpp は yaml-cpp 依存。本体 executable と同じく test binary にも明示 link
  # (#149 で見落とし、別 PC clean build で undefined reference 発覚 → hotfix)。
  # NOTE: humble の `ament_auto_add_gtest` 内で plain signature の target_link_libraries
  # が使われているため、ここも plain (= PUBLIC/PRIVATE/INTERFACE 無し) で揃える必要がある。
  # PRIVATE 化は CMake が「all-keyword or all-plain」混在を error にするため不可。
  target_link_libraries(test_initial_pose_resolver ${_mapoi_yaml_cpp_target})

  # System tag 定義は mapoi_server/system_tags.hpp を source of truth とする。
  # WebUI 側にも同 header との contract test を置き、言語横断の tag drift を検出する (#193)。
  ament_auto_add_gtest(test_system_tags_contract
    test/test_system_tags_contract.cpp
  )

  # Level 2: launch_testing integration tests
  add_launch_test(
    test/test_poi_event_integration.py
  )
  # ROUTE-mode 経路は Nav2 follow_waypoints mock が前提なので別ファイルに分離 (#226)。
  # mock の MultiThreadedExecutor 立ち上げ + 複数 test の cancel/cleanup 待ちで
  # default 60s を超えやすい。連続 pause / landmark コントラスト test 追加 (#236) で
  # 7 test に増えたため 120s から 180s へ延長。
  add_launch_test(
    test/test_poi_event_route_integration.py
    TIMEOUT 180
  )
  # mapoi 主導 waypoint 到達モード (#243) は NavigateToPose mock 前提で別ファイルに分離。
  add_launch_test(
    test/test_poi_event_route_mapoi_arrival.py
    TIMEOUT 120
  )
  add_launch_test(
    test/test_reload_clears_initialpose.py
  )
  add_launch_test(
    test/test_select_map_no_nav2.py
  )
  add_launch_test(
    test/test_backend_status_no_nav2.py
  )
  add_launch_test(
    test/test_localization_backend_status_no_amcl.py
  )

  # Level 3: pytest contract test (#208). In-process pub/sub なので launch_testing 不要。
  ament_add_pytest_test(test_backend_status_liveliness
    test/test_backend_status_liveliness.py
    TIMEOUT 30
  )

  # Install test config
  install(
    FILES test/test_mapoi_config.yaml
    DESTINATION share/${PROJECT_NAME}/test
  )
  install(
    DIRECTORY test/select_map_maps
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
