cmake_minimum_required(VERSION 3.8)
project(portal_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IdClassVec.msg"
  "msg/IdPosTime.msg"
  "msg/IdPosVelTime.msg"
  "msg/IdSample.msg"
  "msg/PosVelClass.msg"
  "msg/RobotCmd.msg"
  "msg/RobotPos.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
