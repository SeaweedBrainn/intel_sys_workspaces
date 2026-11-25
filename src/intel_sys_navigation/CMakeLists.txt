cmake_minimum_required(VERSION 3.8)
project(intel_sys_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/frontier_explorer scripts/tag_follower DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_frontiers test/test_frontiers.py)
  ament_add_pytest_test(test_tag test/test_tag.py)
  ament_add_pytest_test(test_follow test/test_follow.py)
endif()

install(
  DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
