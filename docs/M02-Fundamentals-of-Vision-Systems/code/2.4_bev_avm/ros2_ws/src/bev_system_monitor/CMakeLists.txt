cmake_minimum_required(VERSION 3.8)
project(bev_system_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bev_interfaces REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

add_executable(bev_system_monitor src/system_monitor_node.cpp)
ament_target_dependencies(bev_system_monitor rclcpp bev_interfaces autoware_perception_msgs)
install(TARGETS bev_system_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()