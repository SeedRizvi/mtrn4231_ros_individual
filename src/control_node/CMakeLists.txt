cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

add_executable(supervisor_sub src/supervisor_sub.cpp)
ament_target_dependencies(supervisor_sub rclcpp std_msgs custom_msgs)

target_include_directories(supervisor_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  supervisor_sub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.hpp"
  PATTERN "*.h"
  # Add other patterns for specific header file extensions if needed
)

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
