cmake_minimum_required(VERSION 3.8)
project(mapoi_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointOfInterest.msg"
  "srv/SwitchMap.srv"
  "srv/GetMapInfo.srv"
  "srv/GetTaggedPois.srv"
  "srv/GetRoutePois.srv"
  "action/FollowPoiList.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
