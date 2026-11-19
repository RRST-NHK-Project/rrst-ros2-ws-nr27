cmake_minimum_required(VERSION 3.8)
project(br_swervedrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 行列・運動学のライブラリ(include/br_swervedrive/*.h + src/*.cpp)
# 使う側: #include "br_swervedrive/matrix.h" / "br_swervedrive/zakinema.h" して、
#         target_link_libraries(<ノード名> br_swervedrive_lib) する。
add_library(br_swervedrive_lib
  src/matrix.cpp
  src/zakinema.cpp
)
target_include_directories(br_swervedrive_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(br_swervedrive_lib PUBLIC cxx_std_17)
ament_target_dependencies(
  br_swervedrive_lib
  "rclcpp"
  "geometry_msgs"
)

# 独ステ1ユニット(操舵DM2325 + 駆動DM3519)をPS4で動かすノード
add_executable(infinite_ctrl_node src/infinite_ctrl_node.cpp)
target_compile_features(infinite_ctrl_node PUBLIC cxx_std_17)
ament_target_dependencies(
  infinite_ctrl_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
target_link_libraries(infinite_ctrl_node br_swervedrive_lib)

# 他のパッケージからも使えるように書き出す
install(TARGETS br_swervedrive_lib
  EXPORT export_br_swervedrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
ament_export_targets(export_br_swervedrive HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)
install(TARGETS infinite_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
