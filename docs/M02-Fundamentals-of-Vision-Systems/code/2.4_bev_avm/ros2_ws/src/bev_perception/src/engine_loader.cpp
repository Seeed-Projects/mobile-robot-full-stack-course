#include "bev_perception/engine_loader.hpp"

#include <fstream>
#include <sstream>

#include <NvInferVersion.h>

namespace bev::perception
{

std::shared_ptr<EngineLifetime> loadEngine(
  const std::string & path, nvinfer1::ILogger & logger, std::string * error)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.good()) {
    if (error) *error = "engine file not found: " + path;
    return nullptr;
  }
  std::stringstream ss;
  ss << file.rdbuf();
  const std::string data = ss.str();

  auto lifetime = std::make_shared<EngineLifetime>();
  lifetime->runtime.reset(nvinfer1::createInferRuntime(logger));
  if (!lifetime->runtime) {
    if (error) *error = "createInferRuntime failed";
    return nullptr;
  }
  lifetime->engine = lifetime->runtime->deserializeCudaEngine(data.data(), data.size());
  if (!lifetime->engine) {
    if (error) *error = "deserializeCudaEngine failed (incompatible or corrupt engine)";
    return nullptr;
  }
  return lifetime;
}

EngineInfo inspectEngine(const nvinfer1::ICudaEngine & engine)
{
  EngineInfo info;
  const int32_t ver = getInferLibVersion();  // encoding: major*10000+minor*100+patch
  info.trt_version = std::to_string(ver / 10000) + "." +
                     std::to_string((ver / 100) % 100) + "." +
                     std::to_string(ver % 100);
  info.nb_tensors = engine.getNbIOTensors();
  for (int i = 0; i < info.nb_tensors; ++i) {
    const char * name = engine.getIOTensorName(i);
    if (engine.getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      info.input_names.emplace_back(name);
    } else {
      info.output_names.emplace_back(name);
    }
  }
  // images: engine input 0
  info.images_dims = engine.getTensorShape("images");
  info.images_dtype = engine.getTensorDataType("images");
  info.heatmap_dims = engine.getTensorShape("heatmap_0");
  return info;
}

bool validateEngine(
  const EngineInfo & info, const std::string & expected_trt_major_minor,
  std::string * reason)
{
  // 1. TensorRT version prefix (e.g. "10.3")
  if (info.trt_version.rfind(expected_trt_major_minor, 0) != 0) {
    if (reason) *reason = "TensorRT lib version " + info.trt_version +
      " does not match expected " + expected_trt_major_minor + ".x";
    return false;
  }
  // 2. IO tensor contract (bevdet_one_lt_d engine)
  const int expected = 19;
  if (info.nb_tensors != expected) {
    if (reason) *reason = "engine has " + std::to_string(info.nb_tensors) +
      " tensors, expected " + std::to_string(expected);
    return false;
  }
  if (info.input_names.size() != 12) {
    if (reason) *reason = "engine input count mismatch (expected 12)";
    return false;
  }
  // 3. images tensor: [6,3,900,400] INT32 (byte-carrier for uint8 [6,3,900,1600])
  if (info.images_dims.nbDims != 4 ||
      info.images_dims.d[0] != 6 || info.images_dims.d[1] != 3 ||
      info.images_dims.d[2] != 900 || info.images_dims.d[3] != 400) {
    if (reason) *reason = "images tensor shape mismatch: " +
      std::to_string(info.images_dims.nbDims) + "d [" +
      std::to_string(info.images_dims.d[0]) + "," +
      std::to_string(info.images_dims.d[1]) + "," +
      std::to_string(info.images_dims.d[2]) + "," +
      std::to_string(info.images_dims.d[3]) + "] (expected [6,3,900,400])";
    return false;
  }
  if (info.images_dtype != nvinfer1::DataType::kINT32) {
    if (reason) *reason = "images tensor dtype mismatch (expected INT32 carrier)";
    return false;
  }
  // 4. heatmap output: [1,10,128,128]
  if (info.heatmap_dims.nbDims != 4 ||
      info.heatmap_dims.d[1] != 10 || info.heatmap_dims.d[2] != 128 ||
      info.heatmap_dims.d[3] != 128) {
    if (reason) *reason = "heatmap_0 shape mismatch (expected [1,10,128,128])";
    return false;
  }
  return true;
}

}  // namespace bev::perception