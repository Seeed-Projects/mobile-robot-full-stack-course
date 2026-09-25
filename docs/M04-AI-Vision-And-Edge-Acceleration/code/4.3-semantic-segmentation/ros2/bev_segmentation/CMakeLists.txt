cmake_minimum_required(VERSION 3.16)
project(bev_segmentation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- ros dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- TensorRT + CUDA ---
find_package(CUDA REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NAMES NvInfer.h REQUIRED)
find_library(TENSORRT_INFER_LIB  NAMES nvinfer    REQUIRED)
find_library(TENSORRT_ONNX_LIB   NAMES nvonnxparser REQUIRED)

# --- 头文件 ---
include_directories(
  include
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

# --- 主库: 让 GTest 也能复用 ---
add_library(bev_segmentation_core
  src/segmentation_engine.cpp
  src/preprocess.cpp
  src/postprocess.cpp
)
target_link_libraries(bev_segmentation_core PUBLIC
  ${TENSORRT_INFER_LIB}
  ${TENSORRT_ONNX_LIB}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_cudart_static_LIBRARY}
)

# --- ROS node 可执行 ---
add_executable(segmentation_node src/segmentation_node.cpp)
target_link_libraries(segmentation_node PRIVATE
  bev_segmentation_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# --- 离线 parity 工具 ---
# Not a test: it takes arguments and is driven by
# scripts/m4/verify_segformer_parity.py. It deliberately links the SAME
# bev_segmentation_core the ROS node uses, so a green parity result covers the
# shipped inference path rather than a reimplementation of it.
add_executable(parity_runner test/parity_runner.cpp)
target_link_libraries(parity_runner PRIVATE bev_segmentation_core)

# --- GTest ---
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  include_directories(include)

  # ament_add_gmock 内部 add_executable + target_link_libraries(... plain ...)
  # 我们后续调用必须保持 plain 形式 (不带 PUBLIC/PRIVATE keyword).
  ament_add_gmock(test_postprocessing    test/test_postprocessing.cpp)
  ament_add_gmock(test_drivable_mask     test/test_drivable_mask.cpp)
  ament_add_gmock(test_engine_smoke      test/test_engine_smoke.cpp)
  ament_add_gmock(test_segmentation_node test/test_segmentation_node.cpp)
  ament_add_gmock(test_unletterbox       test/test_unletterbox.cpp)
  ament_add_gmock(test_rate_limiter      test/test_rate_limiter.cpp)

  target_link_libraries(test_postprocessing     bev_segmentation_core)
  target_link_libraries(test_drivable_mask      bev_segmentation_core)
  target_link_libraries(test_unletterbox        bev_segmentation_core)
  target_link_libraries(test_engine_smoke       bev_segmentation_core rclcpp::rclcpp ${sensor_msgs_TARGETS})
  target_link_libraries(test_segmentation_node  bev_segmentation_core rclcpp::rclcpp ${sensor_msgs_TARGETS})

  # --- 标签: 单元测试 vs 模型 artifact gate ---
  # `-L unit` runs everything that needs no model artifact (fast, always valid).
  # `-L engine_gate` is the release gate and is RED until the engine exists.
  set_tests_properties(test_postprocessing    PROPERTIES LABELS "unit")
  set_tests_properties(test_drivable_mask     PROPERTIES LABELS "unit")
  set_tests_properties(test_segmentation_node PROPERTIES LABELS "unit")
  set_tests_properties(test_unletterbox       PROPERTIES LABELS "unit")
  set_tests_properties(test_rate_limiter      PROPERTIES LABELS "unit")
  set_tests_properties(test_engine_smoke      PROPERTIES LABELS "unit;engine_gate")

  # test_engine_smoke 在 engine 缺失时 GTEST_SKIP, 而 gtest 把 skip 记成 pass;
  # 这个 gate 让"engine 不存在"变成 FAIL, 而不是一片绿.
  # Resolve the REAL package directory before walking up. This package is built
  # through the src symlink, and from there the extra path components
  # make a plain "../../.." land on the repository root instead of this module.
  get_filename_component(_m4_pkg_real "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
  get_filename_component(M4_MODULE_ROOT "${_m4_pkg_real}/../../.." ABSOLUTE)
  set(SEGFORMER_ENGINE_PATH
      "${M4_MODULE_ROOT}/models/m4/segmentation/engines/segformer_b0_fp16.engine")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/test/check_engine_gate.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/check_engine_gate.sh"
    @ONLY)
  add_test(NAME engine_existence_gate
           COMMAND bash "${CMAKE_CURRENT_BINARY_DIR}/check_engine_gate.sh")
  set_tests_properties(engine_existence_gate PROPERTIES LABELS "engine_gate")

  # install smoke test scripts
  install(PROGRAMS
    test/image_republisher.py
    DESTINATION lib/${PROJECT_NAME}/test
  )
endif()

# --- install ---
install(TARGETS
  segmentation_node
  parity_runner
  bev_segmentation_core
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/bev_segmentation DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
