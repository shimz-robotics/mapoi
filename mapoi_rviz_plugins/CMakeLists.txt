cmake_minimum_required(VERSION 3.5)
project(mapoi_rviz_plugins)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Qt settings
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

set(HEADERS
  include/mapoi_rviz_plugins/mapoi_panel.hpp
  include/mapoi_rviz_plugins/poi_editor.hpp
  include/mapoi_rviz_plugins/mapoi_pose_tool.hpp
)
qt5_wrap_ui(UIC_FILES
  src/ui/mapoi_panel.ui
  src/ui/poi_editor.ui
)
set(SOURCE_FILES
  src/mapoi_panel.cpp
  src/poi_editor.cpp
  src/mapoi_pose_tool.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})

# 将来のROS 2（Kilted Kaiju以降）と互換性を持たせるため
# ament_auto_package()
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
