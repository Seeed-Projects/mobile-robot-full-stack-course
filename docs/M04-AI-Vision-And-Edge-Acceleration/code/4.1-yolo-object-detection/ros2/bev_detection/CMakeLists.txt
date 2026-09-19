cmake_minimum_required(VERSION 3.8)
project(bev_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()

# =============================================================================
# ROS2 / ament
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(bev_interfaces REQUIRED)

# =============================================================================
# CUDA / TensorRT
# =============================================================================
find_package(CUDAToolkit REQUIRED)

# TensorRT 10.3 (system)
find_path(TRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVINFER NAMES nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVONNXPARSER NAMES nvonnxparser PATHS /usr/lib/aarch64-linux-gnu REQUIRED)

enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# =============================================================================
# Targets
# =============================================================================

# ----- yolo_engine (core TensorRT wrapper) -----
add_library(yolo_engine SHARED
  src/yolo_engine.cpp
  src/preprocessing.cpp
  src/postprocessing.cpp)
target_include_directories(yolo_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yolo_engine PUBLIC
  CUDA::cudart
  ${TRT_NVINFER}
  ${TRT_NVONNXPARSER}
  cv_bridge::cv_bridge)
target_compile_options(yolo_engine PRIVATE -Wno-deprecated-declarations)

# ----- yolo_trt_node (ROS2 node) -----
add_executable(yolo_trt_node src/yolo_trt_node.cpp)
target_include_directories(yolo_trt_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(yolo_trt_node
  rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge bev_interfaces)
target_link_libraries(yolo_trt_node
  ${TRT_NVINFER}
  yolo_engine)
target_compile_options(yolo_trt_node PRIVATE -Wno-deprecated-declarations)

# ----- camera_adapter_node (thin adapter: FrameSet -> front camera Image) -----
add_executable(camera_adapter_node src/camera_adapter_node.cpp)
target_include_directories(camera_adapter_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(camera_adapter_node
  rclcpp sensor_msgs bev_interfaces)
target_compile_options(camera_adapter_node PRIVATE -Wno-deprecated-declarations)

# =============================================================================
# Install
# =============================================================================
install(TARGETS yolo_engine yolo_trt_node camera_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# =============================================================================
# Tests
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Library paths required at test runtime:
  #   - opencv 4.14 build (provides libopencv_*.so.414)
  #   - CUDA 12.6 runtime libs
  #   - TensorRT 10.3 libs (system path)
  set(BEV_DET_TEST_LIBRARY_DIRS
    "/home/seeed/src/opencv-4.14.0-cuda-build/lib"
    "/usr/local/cuda-12.6/lib64"
    "/usr/lib/aarch64-linux-gnu"
  )

  # ---------- Pure-CPU GTest targets (no CUDA, no TRT) ----------------------
  # Note: include path is ${CMAKE_CURRENT_SOURCE_DIR}/include which makes
  # "bev_detection/<header>.hpp" the canonical include prefix.

  # 1) Letterbox math + bbox restore (4:3, 16:9, square, clip).
  ament_add_gtest_executable(test_letterbox test/test_letterbox.cpp)
  target_include_directories(test_letterbox PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_letterbox yolo_engine)
  ament_add_gtest_test(test_letterbox
    APPEND_LIBRARY_DIRS ${BEV_DET_TEST_LIBRARY_DIRS})

  # 2) NMS / IoU / score-gating logic.
  ament_add_gtest_executable(test_postprocessing test/test_postprocessing.cpp)
  target_include_directories(test_postprocessing PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_postprocessing yolo_engine)
  ament_add_gtest_test(test_postprocessing
    APPEND_LIBRARY_DIRS ${BEV_DET_TEST_LIBRARY_DIRS})

  # 3) Detection contract: header, bbox, score, class, empty-frame.
  ament_add_gtest_executable(test_detection_contract test/test_detection_contract.cpp)
  target_include_directories(test_detection_contract PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_detection_contract
    rclcpp sensor_msgs vision_msgs std_msgs)
  target_link_libraries(test_detection_contract yolo_engine)
  ament_add_gtest_test(test_detection_contract
    APPEND_LIBRARY_DIRS ${BEV_DET_TEST_LIBRARY_DIRS})

  # The CUDA/TensorRT-bound smoke test (test_engine_smoke.cpp) is
  # built as a standalone executable outside colcon test because it
  # requires the engine file and a working CUDA device. It is run by
  # scripts/m4/test_yolo_node.sh and the regression gate.
endif()

ament_package()
