cmake_minimum_required(VERSION 3.8)
project(tr_dokusute)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(common REQUIRED)  # PID.hpp (rrst-ros2-ws/common)

# ROSに依存しない計算部分 (単体テストする)
add_library(swerve_kinematics STATIC src/swerve_kinematics.cpp)
target_include_directories(swerve_kinematics PUBLIC include)

# 入力
add_executable(tr_joy_teleop src/tr_joy_teleop_node.cpp)
ament_target_dependencies(tr_joy_teleop rclcpp sensor_msgs geometry_msgs)

# 計算
add_executable(tr_swerve_kinematics src/tr_swerve_kinematics_node.cpp)
target_link_libraries(tr_swerve_kinematics swerve_kinematics)
ament_target_dependencies(tr_swerve_kinematics rclcpp geometry_msgs std_msgs)

add_executable(tr_pid src/tr_pid_node.cpp)
target_include_directories(tr_pid PRIVATE include)
ament_target_dependencies(tr_pid rclcpp std_msgs common)

add_executable(dob_node src/dob_node.cpp)
ament_target_dependencies(dob_node rclcpp geometry_msgs nav_msgs)

# 機構 (ros2can 経由でモーターを動かす)
add_executable(tr_steer_dm2325 src/tr_steer_dm2325_node.cpp)
target_include_directories(tr_steer_dm2325 PRIVATE include)
ament_target_dependencies(tr_steer_dm2325 rclcpp std_msgs std_srvs)

add_executable(tr_drive_735 src/tr_drive_735_node.cpp)
target_include_directories(tr_drive_735 PRIVATE include)
ament_target_dependencies(tr_drive_735 rclcpp std_msgs)

# センサ (確認用)
add_executable(tr_photo_sensor src/tr_photo_sensor_node.cpp)
target_include_directories(tr_photo_sensor PRIVATE include)
ament_target_dependencies(tr_photo_sensor rclcpp std_msgs)

install(TARGETS
  tr_joy_teleop
  tr_swerve_kinematics
  tr_pid
  dob_node
  tr_steer_dm2325
  tr_drive_735
  tr_photo_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve_kinematics test/test_swerve_kinematics.cpp)
  target_link_libraries(test_swerve_kinematics swerve_kinematics)
  ament_add_gtest(test_damiao_slots test/test_damiao_slots.cpp)
  target_include_directories(test_damiao_slots PRIVATE include)

  # copyright: 著作権表記の方針がまだ決まっていないので使わない
  # uncrustify: ROS 2 スタイルを強制し、Google C++ Style と衝突するので使わない
  #   (書式は .clang-format (Google)、規約は下の cpplint (Google製) で見る)
  # cpplint: 著作権チェックだけ外して下で個別に呼ぶ
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright ament_cmake_uncrustify ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")
endif()

ament_package()
