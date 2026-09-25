// YOLO TensorRT engine wrapper implementation.

#include "bev_detection/yolo_engine.hpp"
#include "bev_detection/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace bev::detection
{

// ============================================================================
// YoloEngine
// ============================================================================

YoloEngine::~YoloEngine()
{
  // TensorRT 10: ICudaEngine/IExecutionContext are owned via raw pointers;
  // explicit destroy() was removed. We just delete them.
  delete context_;
  context_ = nullptr;
  delete engine_;
  engine_ = nullptr;
  runtime_.reset();
  if (d_buffer_) {
    cudaFree(d_buffer_);
    d_buffer_ = nullptr;
  }
  delete[] bindings_;
  bindings_ = nullptr;
}

bool YoloEngine::load(const std::string & engine_path, Logger & logger)
{
  logger_ = logger;

  // 1. Load engine file into memory
  std::ifstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    std::fprintf(stderr, "YoloEngine: engine file not found: %s\n", engine_path.c_str());
    return false;
  }
  std::stringstream ss;
  ss << file.rdbuf();
  const std::string engine_data = ss.str();
  file.close();

  // 2. Create runtime
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    std::fprintf(stderr, "YoloEngine: createInferRuntime failed\n");
    return false;
  }

  // 3. Deserialize engine
  engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
  if (!engine_) {
    std::fprintf(stderr, "YoloEngine: deserializeCudaEngine failed (incompatible or corrupt engine)\n");
    return false;
  }

  // 4. Create execution context
  context_ = engine_->createExecutionContext();
  if (!context_) {
    std::fprintf(stderr, "YoloEngine: createExecutionContext failed\n");
    return false;
  }

  // 5. Allocate GPU buffer
  if (!allocateBuffers()) {
    return false;
  }

  std::fprintf(stderr, "YoloEngine: loaded successfully (%s)\n", engine_path.c_str());
  return true;
}

nvinfer1::Dims YoloEngine::getInputDims() const
{
  if (!engine_ || input_name_.empty()) {
    nvinfer1::Dims dims{};
    dims.nbDims = 0;
    return dims;
  }
  return engine_->getTensorShape(input_name_.c_str());
}

bool YoloEngine::allocateBuffers()
{
  if (!engine_) return false;

  // Get tensor names and dimensions
  const int nb_io = engine_->getNbIOTensors();
  size_t total_bytes = 0;

  for (int i = 0; i < nb_io; ++i) {
    const char * name = engine_->getIOTensorName(i);
    nvinfer1::Dims dims = engine_->getTensorShape(name);
    nvinfer1::DataType dtype = engine_->getTensorDataType(name);
    size_t elem_size = 1;
    switch (dtype) {
      case nvinfer1::DataType::kFLOAT: elem_size = 4; break;
      case nvinfer1::DataType::kHALF: elem_size = 2; break;
      case nvinfer1::DataType::kINT8: elem_size = 1; break;
      case nvinfer1::DataType::kINT32: elem_size = 4; break;
      default: elem_size = 4; break;
    }

    size_t size = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      size *= dims.d[d];
    }

    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      input_name_ = name;
      input_size_ = static_cast<int>(size);
      input_dtype_ = dtype;
    } else {
      output_name_ = name;
      output_size_ = static_cast<int>(size);
      output_dtype_ = dtype;
    }

    total_bytes += size * elem_size;
  }

  // Allocate single GPU buffer for input + output
  if (cudaMalloc(&d_buffer_, total_bytes) != cudaSuccess) {
    std::fprintf(stderr, "YoloEngine: cudaMalloc failed (%zu bytes)\n", total_bytes);
    return false;
  }

  // Create bindings array for executeV2
  bindings_ = new void*[nb_io];

  // Set tensor addresses for TensorRT 10.x
  // Convention: inputs first, then outputs
  char * ptr = static_cast<char *>(d_buffer_);
  for (int i = 0; i < nb_io; ++i) {
    const char * name = engine_->getIOTensorName(i);
    nvinfer1::Dims dims = engine_->getTensorShape(name);
    size_t size = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      size *= dims.d[d];
    }
    size_t elem_size = (engine_->getTensorDataType(name) == nvinfer1::DataType::kHALF) ? 2 : 4;

    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      bindings_[0] = ptr;
      input_ptr_ = ptr;
    } else {
      bindings_[1] = ptr;
      output_ptr_ = ptr;
    }

    context_->setTensorAddress(name, ptr);
    ptr += size * elem_size;
  }

  return true;
}

bool YoloEngine::infer(const float * input, float * output, float * inference_ms)
{
  if (!context_ || !d_buffer_ || !bindings_) {
    std::fprintf(stderr, "YoloEngine: engine not ready\n");
    return false;
  }

  // Copy input to GPU
  size_t input_bytes = input_size_ * sizeof(float);
  if (cudaMemcpy(input_ptr_, input, input_bytes, cudaMemcpyHostToDevice) != cudaSuccess) {
    std::fprintf(stderr, "YoloEngine: cudaMemcpy H2D failed\n");
    return false;
  }

  // Execute inference
  bool success = context_->executeV2(bindings_);

  if (!success) {
    std::fprintf(stderr, "YoloEngine: inference execution failed\n");
    return false;
  }

  // Copy output from GPU
  if (cudaMemcpy(output, output_ptr_, output_size_ * sizeof(float),
                 cudaMemcpyDeviceToHost) != cudaSuccess) {
    std::fprintf(stderr, "YoloEngine: cudaMemcpy D2H failed\n");
    return false;
  }

  if (inference_ms) *inference_ms = 0.f;  // use cudaEvent for proper timing
  return true;
}

bool YoloEngine::inferFromImage(
  const uint8_t * h_image, int img_h, int img_w,
  float * output, float * inference_ms, LetterBox * letterbox)
{
  if (!context_ || !d_buffer_ || !bindings_) {
    std::fprintf(stderr, "YoloEngine: engine not ready\n");
    return false;
  }

  // Get model input size from engine
  nvinfer1::Dims input_dims = getInputDims();
  if (input_dims.nbDims != 4) {
    std::fprintf(stderr, "YoloEngine: unexpected input dims %d\n", input_dims.nbDims);
    return false;
  }

  const int model_h = input_dims.d[2];
  const int model_w = input_dims.d[3];

  // Compute letterbox parameters
  float scale = std::min(static_cast<float>(model_w) / img_w,
                         static_cast<float>(model_h) / img_h);
  int resized_h = static_cast<int>(std::round(img_h * scale));
  int resized_w = static_cast<int>(std::round(img_w * scale));
  int pad_h = (model_h - resized_h) / 2;
  int pad_w = (model_w - resized_w) / 2;

  if (letterbox) {
    letterbox->scale = scale;
    letterbox->pad_h = pad_h;
    letterbox->pad_w = pad_w;
  }

  // Prepare input buffer on GPU
  const size_t input_bytes = input_size_ * sizeof(float);
  float * d_input = static_cast<float *>(input_ptr_);

  // Build source BGR cv::Mat from h_image buffer (no copy)
  cv::Mat src_bgr(img_h, img_w, CV_8UC3, const_cast<uint8_t *>(h_image));

  // Letterbox resize: fit scale + grey padding to model input size
  // (resized_h / resized_w / pad_h / pad_w already declared above)

  cv::Mat dst_bgr(model_h, model_w, CV_8UC3, cv::Scalar(114, 114, 114));
  if (resized_h > 0 && resized_w > 0) {
    cv::Mat resized;
    cv::Mat roi(dst_bgr, cv::Rect(pad_w, pad_h, resized_w, resized_h));
    cv::resize(src_bgr, resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);
    resized.copyTo(roi);
  }

  // BGR -> RGB Planar float [C,H,W] using OpenCV split + multiply
  // Fast: no pixel-level loops; relies on NEON/SIMD inside OpenCV
  const float inv_255 = 1.f / 255.f;
  std::vector<float> h_input(input_size_, 0.f);
  {
    cv::Mat rgb_interleaved;
    cv::cvtColor(dst_bgr, rgb_interleaved, cv::COLOR_BGR2RGB);
    std::vector<cv::Mat> planes(3);
    cv::split(rgb_interleaved, planes);
    // Each plane is [H, W] uint8. Normalize to [0,1] float.
    cv::Mat f_r, f_g, f_b;
    planes[0].convertTo(f_r, CV_32FC1, inv_255);
    planes[1].convertTo(f_g, CV_32FC1, inv_255);
    planes[2].convertTo(f_b, CV_32FC1, inv_255);
    // Stack into CHW layout: [R-plane, G-plane, B-plane]
    const int plane_sz = model_h * model_w;
    std::memcpy(&h_input[0 * plane_sz], f_r.ptr<float>(), plane_sz * sizeof(float));
    std::memcpy(&h_input[1 * plane_sz], f_g.ptr<float>(), plane_sz * sizeof(float));
    std::memcpy(&h_input[2 * plane_sz], f_b.ptr<float>(), plane_sz * sizeof(float));
  }

  // Upload to GPU
  if (cudaMemcpy(d_input, h_input.data(), input_bytes, cudaMemcpyHostToDevice) != cudaSuccess) {
    std::fprintf(stderr, "YoloEngine: input upload failed\n");
    return false;
  }

  // Time the inference
  cudaEvent_t start, end;
  cudaEventCreate(&start);
  cudaEventCreate(&end);
  cudaEventRecord(start);

  // Execute inference
  bool success = context_->executeV2(bindings_);

  cudaEventRecord(end);
  cudaEventSynchronize(end);

  float ms = 0.f;
  cudaEventElapsedTime(&ms, start, end);
  cudaEventDestroy(start);
  cudaEventDestroy(end);

  if (!success) {
    std::fprintf(stderr, "YoloEngine: inference failed\n");
    return false;
  }

  // Download output
  if (cudaMemcpy(output, output_ptr_, output_size_ * sizeof(float),
                 cudaMemcpyDeviceToHost) != cudaSuccess) {
    std::fprintf(stderr, "YoloEngine: output download failed\n");
    return false;
  }

  if (inference_ms) *inference_ms = ms;

  return true;
}

// ============================================================================
// YoloInferencer
// ============================================================================

bool YoloInferencer::init(const Config & config, Logger & logger)
{
  config_ = config;

  engine_ = std::make_unique<YoloEngine>();
  if (!engine_->load(config.engine_path, logger)) {
    std::fprintf(stderr, "YoloInferencer: failed to load engine\n");
    return false;
  }

  postprocess_ = std::make_unique<YoloPostprocess>(
    config.num_classes, config.conf_thresh, config.nms_thresh);

  // Pre-allocate output buffer once. YOLO11n output: [1, 84, 8400].
  // Sized for the worst-case (num_classes + 4) * 8400 to stay forward-safe
  // with sibling YOLO models; current model uses 84.
  const int output_floats = (4 + config.num_classes) * 8400;
  output_buffer_.assign(static_cast<size_t>(output_floats), 0.f);

  return true;
}

YoloResult YoloInferencer::detect(const uint8_t * h_image, int img_h, int img_w)
{
  YoloResult result;

  if (!engine_ || !postprocess_ || output_buffer_.empty()) {
    std::fprintf(stderr, "YoloInferencer: not initialized\n");
    return result;
  }

  // Reuse pre-allocated buffer (no per-call allocation).
  float inference_ms = 0.f;
  LetterBox letterbox;

  if (!engine_->inferFromImage(h_image, img_h, img_w,
                               output_buffer_.data(), &inference_ms, &letterbox)) {
    std::fprintf(stderr, "YoloInferencer: inference failed\n");
    return result;
  }

  result.inference_ms = inference_ms;
  result.letterbox = letterbox;
  result.boxes = postprocess_->parse(
    output_buffer_.data(), static_cast<int>(output_buffer_.size()),
    img_h, img_w, letterbox);

  return result;
}

void YoloInferencer::setPostprocessThresholds(float conf_thresh, float nms_thresh)
{
  config_.conf_thresh = conf_thresh;
  config_.nms_thresh = nms_thresh;
  postprocess_ = std::make_unique<YoloPostprocess>(
    config_.num_classes, config_.conf_thresh, config_.nms_thresh);
}

// ============================================================================
// ClassNames
// ============================================================================

bool ClassNames::load(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::fprintf(stderr, "ClassNames: cannot open %s\n", path.c_str());
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Remove trailing whitespace
    while (!line.empty() && std::isspace(line.back())) line.pop_back();
    if (!line.empty()) {
      names.push_back(line);
    }
  }
  file.close();

  std::fprintf(stderr, "ClassNames: loaded %zu classes from %s\n", names.size(), path.c_str());
  return !names.empty();
}

const std::string & ClassNames::get(int id) const
{
  static const std::string unknown = "unknown";
  if (id >= 0 && id < static_cast<int>(names.size())) {
    return names[id];
  }
  return unknown;
}

}  // namespace bev::detection
