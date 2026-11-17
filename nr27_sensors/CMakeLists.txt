cmake_minimum_required(VERSION 3.8)
project(nr27_sensors)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config udev DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright: 著作権表記の方針がまだ決まっていないので使わない
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
