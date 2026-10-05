cmake_minimum_required(VERSION 3.8)
project(spider_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

# 手动调试节点：绕过数学链，直接指定 18 路舵机角。
# 需要 ament_index_cpp 来定位 spider 包的 config/default_pose.yaml。
add_executable(manual_servo_node
  src/manual_servo.cpp
)

ament_target_dependencies(manual_servo_node
  ament_index_cpp
  rclcpp
  robot_interfaces
)

# 单腿 IK 控制节点：输入腿局部坐标 (x,y,z)，解算后只更新该腿的 3 路。
# 复用 leg_calc 的运动学核心库（leg_calc_core，由 leg_calc 导出）。
find_package(leg_calc REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(leg_ik_node
  src/leg_ik.cpp
)

target_link_libraries(leg_ik_node
  leg_calc::leg_calc_core
  Eigen3::Eigen
)

ament_target_dependencies(leg_ik_node
  ament_index_cpp
  rclcpp
  robot_interfaces
)

install(TARGETS
  spider_task_node
  servo_listener_node
  manual_servo_node
  leg_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
