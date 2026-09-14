#pragma once
// TensorRT engine loading + startup validation for bevdet_node.
// Hard-fails on mismatch: version, IO tensor count, dtype, resolution.
#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>

namespace bev::perception
{

struct EngineInfo
{
  std::string trt_version;             // from libnvinfer (getInferLibVersion)
  int nb_tensors{0};
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  nvinfer1::Dims images_dims{};        // input "images"
  nvinfer1::DataType images_dtype{nvinfer1::DataType::kFLOAT};
  nvinfer1::Dims heatmap_dims{};       // output "heatmap_0"
  std::string error;
};

struct EngineLifetime
{
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  nvinfer1::ICudaEngine * engine{nullptr};
};

/// Load and deserialize engine file. Returns nullptr on failure (error set).
std::shared_ptr<EngineLifetime> loadEngine(
  const std::string & path, nvinfer1::ILogger & logger, std::string * error);

/// Inspect a deserialized engine.
EngineInfo inspectEngine(const nvinfer1::ICudaEngine & engine);

/// Startup validation against the deployed model contract (engine contract for
/// bevdet_one_lt_d: 19 tensors, images INT32 [6,3,900,400], heatmap [1,10,128,128]).
/// Returns true when compatible; text reason otherwise.
bool validateEngine(
  const EngineInfo & info, const std::string & expected_trt_major_minor,
  std::string * reason);

}  // namespace bev::perception