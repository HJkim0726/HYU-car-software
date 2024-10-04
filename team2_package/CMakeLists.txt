cmake_minimum_required(VERSION 3.8)
project(team2_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(interface REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${interface_INCLUDE_DIRS}
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

add_executable(color_changer src/color_changer.cpp)
ament_target_dependencies(color_changer rclcpp turtlesim interface)

add_executable(hmi src/hmi.cpp)
ament_target_dependencies(hmi rclcpp turtlesim interface visualization_msgs)

add_executable(moving_turtle src/moving_turtle.cpp)
ament_target_dependencies(moving_turtle rclcpp turtlesim geometry_msgs)

install(TARGETS
  color_changer
  hmi
  moving_turtle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
