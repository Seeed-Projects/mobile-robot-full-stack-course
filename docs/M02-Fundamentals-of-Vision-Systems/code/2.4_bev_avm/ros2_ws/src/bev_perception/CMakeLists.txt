cmake_minimum_required(VERSION 3.8)
project(bev_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bev_interfaces REQUIRED)
find_package(bev_preprocessor REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
# NOTE: no OpenCV dependency here; debug image dumps use PPM (see bevdet_node.cpp)
find_library(JPEG_LIB NAMES jpeg REQUIRED)

enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# TensorRT 10.3 (system)
find_path(TRT_INCLUDE_DIR NvInfer.h PATHS /usr/include/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVINFER NAMES nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
find_library(TRT_NVONNXPARSER NAMES nvonnxparser PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
find_path(THRUST_INCLUDE_DIR thrust/version.h PATHS /usr/local/cuda-12.6/include REQUIRED)

# ---- vendored BEVDet core (CUDA sources from bevdet_vendor) ----------------
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../bevdet_vendor)
set(BEVDET_CORE_SOURCES
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

add_library(bevdet_core SHARED ${BEVDET_CORE_SOURCES})
target_include_directories(bevdet_core PUBLIC
  ${VENDOR_DIR}/include
  ${TRT_INCLUDE_DIR}
  ${THRUST_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(bevdet_core PUBLIC
  CUDA::cudart
  ${TRT_NVINFER}
  ${TRT_NVONNXPARSER}
  yaml-cpp
  ${JPEG_LIB})
target_compile_options(bevdet_core PRIVATE -Wno-deprecated-declarations)
target_compile_definitions(bevdet_core PRIVATE FP16_PRECISION_DEFAULT=1)
set_target_properties(bevdet_core PROPERTIES CUDA_ARCHITECTURES 87)

add_library(bev_engine_loader SHARED src/engine_loader.cpp)
target_include_directories(bev_engine_loader PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bev_engine_loader PUBLIC ${TRT_NVINFER} CUDA::cudart)

add_executable(bevdet_node src/bevdet_node.cpp)
ament_target_dependencies(bevdet_node
  rclcpp sensor_msgs std_msgs tf2 tf2_ros bev_interfaces
  bev_preprocessor autoware_perception_msgs)
target_link_libraries(bevdet_node
  bevdet_core
  bev_engine_loader
  CUDA::cudart
  ${TRT_NVINFER}
  yaml-cpp)
target_include_directories(bevdet_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(bevdet_node PRIVATE -Wno-deprecated-declarations)

install(TARGETS bevdet_node bevdet_core bev_engine_loader
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()