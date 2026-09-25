#pragma once
// YOLO TensorRT engine wrapper for bev_detection.
// Uses TensorRT 10.3 API with getNbIOTensors / executeV2 + bindings.

#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <NvInfer.h>

#include "types.hpp"
#include "postprocessing.hpp"

namespace bev::detection
{

/// Logger compatible with TensorRT 10.x ILogger interface.
class Logger : public nvinfer1::ILogger
{
public:
  explicit Logger(nvinfer1::ILogger::Severity severity = nvinfer1::ILogger::Severity::kWARNING)
    : reportable_severity_(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity > reportable_severity_) return;
    const char * prefix = "TRT: ";
    switch (severity) {
      case Severity::kINTERNAL_ERROR: prefix = "TRT [ERROR]: "; break;
      case Severity::kERROR: prefix = "TRT [ERROR]: "; break;
      case Severity::kWARNING: prefix = "TRT [WARN]: "; break;
      case Severity::kINFO: prefix = "TRT [INFO]: "; break;
      default: prefix = "TRT [???]: "; break;
    }
    std::fprintf(stderr, "%s%s\n", prefix, msg);
  }

  nvinfer1::ILogger::Severity reportable_severity_;
};

/// RAII wrapper for TensorRT engine + runtime.
class YoloEngine
{
public:
  YoloEngine() = default;
  ~YoloEngine();

  YoloEngine(const YoloEngine &) = delete;
  YoloEngine & operator=(const YoloEngine &) = delete;

  /// Load engine from file. Returns true on success.
  bool load(const std::string & engine_path, Logger & logger);

  /// Get input dimensions.
  nvinfer1::Dims getInputDims() const;

  /// Run inference on a pre-allocated float buffer (CHW, RGB, normalized 0-1).
  bool infer(const float * input, float * output, float * inference_ms);

  /// Run inference from uint8 BGR host buffer.
  bool inferFromImage(
    const uint8_t * h_image, int img_h, int img_w,
    float * output, float * inference_ms, LetterBox * letterbox);

  /// Check if engine is loaded.
  bool ready() const { return context_ != nullptr; }

  /// Get model input size.
  int inputSize() const { return input_size_; }
  int outputSize() const { return output_size_; }

private:
  bool allocateBuffers();

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};

  std::string input_name_;
  std::string output_name_;
  nvinfer1::DataType input_dtype_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_dtype_{nvinfer1::DataType::kFLOAT};

  int input_size_{0};
  int output_size_{0};

  void * d_buffer_{nullptr};     // input + output combined
  void * input_ptr_{nullptr};    // pointer to input portion of d_buffer
  void * output_ptr_{nullptr};   // pointer to output portion of d_buffer
  void ** bindings_{nullptr};    // bindings array for executeV2
};

/// Full YOLO inference pipeline.
class YoloInferencer
{
public:
  struct Config
  {
    std::string engine_path;
    int input_width{640};
    int input_height{640};
    int num_classes{80};
    float conf_thresh{0.25f};
    float nms_thresh{0.45f};
  };

  YoloInferencer() = default;

  /// Initialize with config.
  bool init(const Config & config, Logger & logger);

  /// Run full inference on BGR image.
  YoloResult detect(const uint8_t * h_image, int img_h, int img_w);

  /// Update only the CPU postprocessor; the TensorRT engine stays loaded.
  void setPostprocessThresholds(float conf_thresh, float nms_thresh);

  bool ready() const { return engine_ != nullptr; }

private:
  Config config_;
  std::unique_ptr<YoloEngine> engine_;
  std::unique_ptr<YoloPostprocess> postprocess_;

  // Pre-allocated output buffer for TensorRT engine output.
  // Sized once at init() to 84 * 8400 (YOLO11n output shape) to avoid
  // per-call heap allocation in the hot path.
  std::vector<float> output_buffer_;
};

}  // namespace bev::detection
