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

add_executable(leg_calc_node
  src/main.cpp
  src/leg_kinematics.cpp
  src/servo18_mapper.cpp
)

target_include_directories(leg_calc_node
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(leg_calc_node
  Eigen3::Eigen
  orocos-kdl
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
endif()

ament_package()
