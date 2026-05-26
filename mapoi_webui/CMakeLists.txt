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
  # yaml_handler._round_poi_floats の桁数契約 pin (#242)。frontend (poi-filter.js
  # roundTo + POSE_*_DIGITS / TOLERANCE_*_DIGITS) と必ず一致させる。
  ament_add_pytest_test(test_yaml_handler_round
    test/test_yaml_handler_round.py
    TIMEOUT 10
  )
  # yaml_handler.compute_config_version の決定性 / 変化検出 pin (#241 楽観的競合検出)。
  ament_add_pytest_test(test_yaml_handler_version
    test/test_yaml_handler_version.py
    TIMEOUT 10
  )
endif()

ament_package()
