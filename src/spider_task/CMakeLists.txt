cmake_minimum_required(VERSION 3.8)
project(spider_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(spider_task_node
  src/main.cpp
)

ament_target_dependencies(spider_task_node
  geometry_msgs
  rclcpp
  robot_interfaces
)

add_executable(servo_listener_node
  src/servo_listener.cpp
)

ament_target_dependencies(servo_listener_node
  rclcpp
  robot_interfaces
)

install(TARGETS
  spider_task_node
  servo_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
