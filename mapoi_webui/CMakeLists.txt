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
  # Flask endpoint レベルでの `/api/pois` POST 楽観的競合検出契約 (#241 / #246)。
  # 純関数 test (test_yaml_handler_version) と分けて、409 + code + current_version 経路を pin。
  ament_add_pytest_test(test_api_save_pois_version_conflict
    test/test_api_save_pois_version_conflict.py
    TIMEOUT 10
  )
  # `get_navigation_capabilities` の subscriber 数→availability 統合契約 (#199)。
  # resolver 単体 (test_resolve_backend_status_for_ui) と endpoint test の中間にある
  # 「legacy fallback vs backend_status 上書き / localization 独立」を pin。
  ament_add_pytest_test(test_get_navigation_capabilities
    test/test_get_navigation_capabilities.py
    TIMEOUT 10
  )
  # navigation API endpoint 契約 (#199): /api/nav/switch-map validation (400/404)、
  # /api/mode・/api/nav/status の payload shape を Flask test_client で固定。
  ament_add_pytest_test(test_api_nav_endpoints
    test/test_api_nav_endpoints.py
    TIMEOUT 10
  )
endif()

ament_package()
