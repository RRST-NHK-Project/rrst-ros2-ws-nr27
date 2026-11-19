cmake_minimum_required(VERSION 3.8)
project(nr27_localization)

find_package(ament_cmake REQUIRED)

# maps/ は dev_ws の world から scripts/import_dev_ws_maps.sh で取り込んだもの
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright: 著作権表記の方針がまだ決まっていないので使わない
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_maps test/test_maps.py)
endif()

ament_package()
