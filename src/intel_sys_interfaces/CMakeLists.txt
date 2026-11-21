cmake_minimum_required(VERSION 3.8)
project(intel_sys_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorsState.msg"
  "msg/LedState.msg"
  "msg/BuzzerState.msg"
  "msg/ButtonState.msg"
  "msg/Sbus.msg"
  "msg/OLEDState.msg"
  "msg/PWMServoState.msg"
  "msg/SetPWMServoState.msg"
  "msg/BusServoState.msg"
  "msg/SetBusServoState.msg"
  "msg/ServoPosition.msg"
  "msg/ServosPosition.msg"
  "msg/GetBusServoCmd.msg"
  "msg/GetPWMServoCmd.msg"
  "srv/GetBusServoState.srv"
  "srv/GetPWMServoState.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
