cmake_minimum_required(VERSION 3.8)
project(bev_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrameSet.msg"
  "msg/SyncStats.msg"
  "msg/SystemStatus.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()