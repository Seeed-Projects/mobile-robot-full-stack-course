# Phase 0 standalone build: BEVDet vendor + bevdemo, no ROS/ament.
# ROS2 packaging of bevdet_vendor (ament) is used from Phase 2 onward.
cmake_minimum_required(VERSION 3.22)
project(bevdet_phase0 LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wno-deprecated-declarations)

find_package(CUDAToolkit REQUIRED)

# TensorRT 10.3 (system): headers /usr/include/aarch64-linux-gnu
find_path(TRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVINFER NAMES nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVONNXPARSER NAMES nvonnxparser PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVPARSE NAMES nvparsers PATHS /usr/lib/aarch64-linux-gnu)

# yaml-cpp, libjpeg, Eigen3
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(JPEG_LIB NAMES jpeg REQUIRED)

# GPU arch: AGX Orin = sm_87
set(CMAKE_CUDA_ARCHITECTURES 87)

set(VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/bevdet_vendor")

set(BEVDET_SOURCES
  ${VENDOR_DIR}/src/bevdet.cpp
  ${VENDOR_DIR}/src/preprocess.cu
  ${VENDOR_DIR}/src/iou3d_nms.cu
  ${VENDOR_DIR}/src/postprocess.cu
  ${VENDOR_DIR}/src/data.cpp
  ${VENDOR_DIR}/src/cpu_jpegdecoder.cpp
  ${VENDOR_DIR}/src/nvjpegdecoder.cpp
  ${VENDOR_DIR}/src/preprocess_plugin.cu
  ${VENDOR_DIR}/src/bevpool_plugin.cu
  ${VENDOR_DIR}/src/alignbev_plugin.cu
  ${VENDOR_DIR}/src/gatherbev_plugin.cu
  ${VENDOR_DIR}/src/precision_config.cpp
)

add_library(bevdet_phase0 SHARED ${BEVDET_SOURCES})
target_include_directories(bevdet_phase0 PUBLIC
  ${VENDOR_DIR}/include
  ${TRT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(bevdet_phase0 PUBLIC
  CUDA::cudart
  ${TRT_NVINFER}
  ${TRT_NVONNXPARSER}
  yaml-cpp
  ${JPEG_LIB})
target_compile_definitions(bevdet_phase0 PRIVATE FP16_PRECISION_DEFAULT=1)

add_executable(bevdemo ${VENDOR_DIR}/demo_bevdet.cpp)
target_link_libraries(bevdemo PRIVATE bevdet_phase0)

add_executable(bevdet_bench ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_bevdet.cpp)
target_link_libraries(bevdet_bench PRIVATE bevdet_phase0)
target_compile_options(bevdet_bench PRIVATE -Wno-deprecated-declarations)

install(TARGETS bevdet_phase0 bevdemo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)