cmake_minimum_required(VERSION 3.8)
project(bev_preprocessor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(bev_preprocessor SHARED src/gpu_image_stager.cpp)
target_include_directories(bev_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bev_preprocessor PUBLIC CUDA::cudart)
target_compile_features(bev_preprocessor PUBLIC cxx_std_17)

install(TARGETS bev_preprocessor EXPORT export_bev_preprocessor
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_bev_preprocessor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(CUDAToolkit)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpu_image_stager test/test_gpu_image_stager.cpp)
  target_link_libraries(test_gpu_image_stager bev_preprocessor CUDA::cudart)
endif()

ament_package()