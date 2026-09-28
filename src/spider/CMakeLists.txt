cmake_minimum_required(VERSION 3.8)
project(spider)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  model
  DESTINATION share/${PROJECT_NAME}
)

# URDF 生成脚本。用 install(PROGRAMS) 而不是 DIRECTORY，是为了让它在安装后
# 仍然带着可执行权限——launch 文件会直接调用它。
install(PROGRAMS
  scripts/generate_urdf.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
