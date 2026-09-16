cmake_minimum_required(VERSION 3.8)
project(bev_camera_sync)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bev_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(camera_sync_node src/camera_sync_node.cpp)
target_include_directories(camera_sync_node PRIVATE include)
ament_target_dependencies(camera_sync_node rclcpp sensor_msgs bev_interfaces cv_bridge)
install(TARGETS camera_sync_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_set_assembler test/test_frame_set_assembler.cpp)
  target_include_directories(test_frame_set_assembler PRIVATE include)
  ament_target_dependencies(test_frame_set_assembler rclcpp sensor_msgs bev_interfaces)
endif()

ament_package()