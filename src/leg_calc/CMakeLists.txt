cmake_minimum_required(VERSION 3.8)
project(leg_calc)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# leg_calc_core：与 ROS 无关的纯数学 / 映射库
#   - 只依赖 Eigen 和 KDL，不依赖 rclcpp，因此可以被单元测试直接链接
#   - 节点和测试都链接它，避免同一批 .cpp 被编译两遍，
#     也保证测试验证的就是实际运行的那份代码
# ---------------------------------------------------------------------------
add_library(leg_calc_core STATIC
  src/demo_chain.cpp
  src/foot_trajectory.cpp
  src/gait_phase_manager.cpp
  src/leg_kinematics.cpp
  src/servo18_mapper.cpp
)

target_include_directories(leg_calc_core
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(leg_calc_core
  Eigen3::Eigen
  orocos-kdl
)

# 节点本体：只负责 ROS 装配（参数、订阅、定时器、发布），数学在库里
add_executable(leg_calc_node
  src/main.cpp
)

target_link_libraries(leg_calc_node
  leg_calc_core
)

ament_target_dependencies(leg_calc_node
  geometry_msgs
  rclcpp
  kdl_parser
  robot_interfaces
)

install(TARGETS
  leg_calc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # 单元测试：直接链接 leg_calc_core，因此测的就是节点实际运行的那份代码。
  # 只测纯逻辑（几何 / 步态 / 映射），不测 ROS 节点与 topic。
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_foot_trajectory test/test_foot_trajectory.cpp)
  target_link_libraries(test_foot_trajectory leg_calc_core)
endif()

ament_package()

