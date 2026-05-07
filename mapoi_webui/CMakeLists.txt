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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # `_resolve_backend_status_for_ui` の状態遷移 unit test (#208, #212 codex review)。
  # mapoi_webui_node の Node 部分には依存しない pure logic。
  ament_add_pytest_test(test_resolve_backend_status_for_ui
    test/test_resolve_backend_status_for_ui.py
    TIMEOUT 10
  )
endif()

ament_package()
