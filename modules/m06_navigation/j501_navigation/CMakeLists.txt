cmake_minimum_required(VERSION 3.16)
project(j501_navigation)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch maps worlds models rviz behavior_trees scenes assets validation
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/global_plan scripts/local_follow scripts/navigate scripts/obstacle scripts/grid_plan scripts/benchmark
  scripts/lidar_perception scripts/survey scripts/save_map scripts/relocalize scripts/localization_monitor scripts/algorithm_view
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md README_zh_CN.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(planning_invariants tests/test_planning.py TIMEOUT 30)
  ament_add_pytest_test(lidar_invariants tests/test_lidar.py TIMEOUT 30)
endif()
ament_package()
