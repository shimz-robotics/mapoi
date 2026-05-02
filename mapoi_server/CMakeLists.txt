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

ament_auto_add_executable(mapoi_nav_server
  src/mapoi_nav_server.cpp
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
  find_package(launch_testing_ament_cmake REQUIRED)

  # Level 1: gtest unit tests
  ament_auto_add_gtest(test_nav_server_unit
    test/test_nav_server_unit.cpp
    src/mapoi_nav_server.cpp
  )
  target_compile_definitions(test_nav_server_unit PRIVATE UNIT_TEST)

  # initial_pose_resolver は純関数なので test には resolver の cpp だけを link する
  # (#150 で mapoi_server.cpp 全体 link から軽量化、Node 依存を test から排除)。
  ament_auto_add_gtest(test_mapoi_server_unit
    test/test_mapoi_server_unit.cpp
    src/initial_pose_resolver.cpp
  )
  target_compile_definitions(test_mapoi_server_unit PRIVATE UNIT_TEST)
  # initial_pose_resolver.cpp は yaml-cpp 依存。本体 executable と同じく test binary にも明示 link
  # (#149 で見落とし、別 PC clean build で undefined reference 発覚 → hotfix)。
  # NOTE: humble の `ament_auto_add_gtest` 内で plain signature の target_link_libraries
  # が使われているため、ここも plain (= PUBLIC/PRIVATE/INTERFACE 無し) で揃える必要がある。
  # PRIVATE 化は CMake が「all-keyword or all-plain」混在を error にするため不可。
  target_link_libraries(test_mapoi_server_unit ${_mapoi_yaml_cpp_target})

  # Level 2: launch_testing integration tests
  add_launch_test(
    test/test_poi_event_integration.py
  )
  add_launch_test(
    test/test_reload_clears_initialpose.py
  )
  add_launch_test(
    test/test_select_map_no_nav2.py
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
