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
# Jazzy 用 (ros_gz_interfaces ベース) は mapoi_gz_bridge として別途 (#42)。
find_package(gazebo_msgs QUIET)
if(gazebo_msgs_FOUND)
  ament_auto_add_executable(mapoi_gazebo_bridge
    src/mapoi_gazebo_bridge.cpp
  )
  target_link_libraries(mapoi_gazebo_bridge ${_mapoi_yaml_cpp_target})
  ament_target_dependencies(mapoi_gazebo_bridge gazebo_msgs)
  message(STATUS "Building mapoi_gazebo_bridge (Gazebo Classic)")
else()
  message(STATUS "Skipping mapoi_gazebo_bridge (gazebo_msgs not found)")
endif()


install(
  DIRECTORY
    launch
    maps
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

  # Level 2: launch_testing integration tests
  add_launch_test(
    test/test_poi_event_integration.py
  )

  # Install test config
  install(
    FILES test/test_mapoi_config.yaml
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
