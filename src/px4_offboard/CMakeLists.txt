cmake_minimum_required(VERSION 3.8)
project(px4_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# 添加可执行文件
add_executable(offboard_control src/offboard_control.cpp)
add_executable(figure8 src/figure8_node.cpp)

# 链接依赖库
ament_target_dependencies(offboard_control rclcpp px4_msgs)
ament_target_dependencies(figure8 rclcpp px4_msgs)

# 安装规则 
install(TARGETS offboard_control figure8
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
