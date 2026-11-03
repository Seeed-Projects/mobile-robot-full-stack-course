cmake_minimum_required(VERSION 3.16)
project(j501_chassis_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch rviz worlds assets validation
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME})
install(FILES README.md README_zh_CN.md DESTINATION share/${PROJECT_NAME})
ament_package()
