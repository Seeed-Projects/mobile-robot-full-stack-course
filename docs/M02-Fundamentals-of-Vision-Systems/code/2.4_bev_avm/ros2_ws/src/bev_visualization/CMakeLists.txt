cmake_minimum_required(VERSION 3.8)
project(bev_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

add_executable(bev_visualization src/visualization_node.cpp)
ament_target_dependencies(bev_visualization rclcpp visualization_msgs tf2 autoware_perception_msgs)
install(TARGETS bev_visualization DESTINATION lib/${PROJECT_NAME})

ament_package()