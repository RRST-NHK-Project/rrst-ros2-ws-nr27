cmake_minimum_required(VERSION 3.8)
project(br_ros2can_gateway)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# 割り当て表と換算、足回りの式（swerve.py）。ROS に依存しない純関数と、その単体試験
install(PROGRAMS
  scripts/gateway_map.py
  scripts/swerve.py
  scripts/verify_swerve.py
  scripts/verify_gateway_map.py
  scripts/swerve_drive.py
  scripts/gateway_node.py
  scripts/ros2can_fake.py
  scripts/verify_gateway_node.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright: 著作権表記の方針がまだ決まっていないので使わない（nr27_localization と同じ）
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  # ROS に依存しない単体試験だけを載せる。ノードを上げる verify_gateway_node.py（約2分）は手で流す
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(verify_gateway_map scripts/verify_gateway_map.py)
  ament_add_pytest_test(verify_swerve scripts/verify_swerve.py)
endif()

ament_package()
