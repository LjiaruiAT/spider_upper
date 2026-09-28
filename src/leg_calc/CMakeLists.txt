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
  src/foot_trajectory.cpp
  src/gait_phase_manager.cpp
  src/leg_chain.cpp
  src/leg_kinematics.cpp
  src/leg_layout.cpp
  src/servo18_mapper.cpp
  src/velocity_smoother.cpp
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
  # 暂时不启用 ament_lint_auto（copyright / cpplint / uncrustify）。
  #
  # 原因：本工程从未按 lint 规范整理过，一旦启用会立刻产生 80+ 个纯风格失败
  # （缺版权头、行超过 100 字符、include 顺序、格式化差异），把真正的单元测试
  # 结果淹没在一个巨大的纯格式 diff 里。
  #
  # 计划：等逻辑稳定后，把"让工程 lint-clean"当成一个独立主题一次做完。
  # 到时把下面两行取消注释即可（package.xml 里的 test_depend 仍然保留）：
  #
  #   find_package(ament_lint_auto REQUIRED)
  #   ament_lint_auto_find_test_dependencies()

  # 单元测试：直接链接 leg_calc_core，因此测的就是节点实际运行的那份代码。
  # 只测纯逻辑（几何 / 步态 / 映射），不测 ROS 节点与 topic。
  find_package(ament_cmake_gtest REQUIRED)

  # 一个模块一个测试文件，全部链接同一个 leg_calc_core。
  # 文件名必须与 test/<name>.cpp 对应。
  set(leg_calc_tests
    test_common_types
    test_foot_trajectory
    test_gait_phase_manager
    test_gait_types
    test_leg_chain
    test_leg_kinematics
    test_leg_layout
    test_servo18_mapper
    test_velocity_smoother
  )

  foreach(test_name IN LISTS leg_calc_tests)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} leg_calc_core)
  endforeach()
endif()

ament_package()

