cmake_minimum_required(VERSION 3.8)
project(nr27_wall_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS 非依存のコア (単体テスト・オフライン検証用に分けてある)
add_library(wall_fitter src/wall_fitter.cc)
target_include_directories(wall_fitter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(wall_detection_node src/wall_detection_node.cc)
target_link_libraries(wall_detection_node wall_fitter)
ament_target_dependencies(wall_detection_node
  rclcpp rcl_interfaces sensor_msgs std_msgs nav_msgs geometry_msgs
  visualization_msgs)

install(TARGETS wall_fitter wall_detection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wall_fitter_test test/wall_fitter_test.cc)
  target_link_libraries(wall_fitter_test wall_fitter)
endif()

ament_package()
