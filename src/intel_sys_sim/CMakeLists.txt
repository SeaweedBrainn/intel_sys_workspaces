cmake_minimum_required(VERSION 3.8)
project(intel_sys_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sim_lidar_time_adapter src/sim_lidar_time_adapter.cpp)
ament_target_dependencies(sim_lidar_time_adapter
  rclcpp
  sensor_msgs
)

install(
  TARGETS sim_lidar_time_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
