// src/segmentation_engine.cpp
//
// TensorRT 10.x 引擎封装实现.

#include "bev_segmentation/segmentation_engine.hpp"

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace bev_segmentation {

namespace {

class StderrLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity == Severity::kWARNING || severity == Severity::kERROR) {
            std::cerr << "[TRT] " << msg << std::endl;
        }
    }
};

StderrLogger& get_logger() {
    static StderrLogger logger;
    return logger;
}

void check_cuda(cudaError_t err, const char* what) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA error at ") + what + ": "
                                 + cudaGetErrorString(err));
    }
}

}  // namespace

SegmentationEngine::SegmentationEngine() = default;

SegmentationEngine::~SegmentationEngine() {
    free_buffers();
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(trt_context_);
    auto* eng = static_cast<nvinfer1::ICudaEngine*>(trt_engine_);
    auto* rt  = static_cast<nvinfer1::IRuntime*>(trt_runtime_);
    delete ctx;
    delete eng;
    delete rt;
}

void SegmentationEngine::load(const std::string& engine_path) {
    if (loaded_) {
        throw std::runtime_error("SegmentationEngine already loaded");
    }

    std::ifstream in(engine_path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open engine file: " + engine_path);
    }
    in.seekg(0, std::ios::end);
    const size_t size = static_cast<size_t>(in.tellg());
    in.seekg(0, std::ios::beg);
    std::vector<char> buf(size);
    if (size > 0 && !in.read(buf.data(), static_cast<std::streamsize>(size))) {
        throw std::runtime_error("Failed to read engine file: " + engine_path);
    }

    auto* runtime = nvinfer1::createInferRuntime(get_logger());
    if (!runtime) {
        throw std::runtime_error("createInferRuntime returned null");
    }
    auto* engine = runtime->deserializeCudaEngine(buf.data(), size);
    if (!engine) {
        delete runtime;
        throw std::runtime_error("deserializeCudaEngine failed");
    }
    auto* context = engine->createExecutionContext();
    if (!context) {
        delete engine;
        delete runtime;
        throw std::runtime_error("createExecutionContext failed");
    }

    trt_runtime_ = runtime;
    trt_engine_  = engine;
    trt_context_ = context;

    check_cuda(cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&stream_)),
               "cudaStreamCreate");

    allocate_buffers();

    loaded_ = true;
    std::cerr << "[SegmentationEngine] loaded: " << engine_path
              << " (input " << input_size()
              << ", output " << output_size() << ")" << std::endl;
}

void SegmentationEngine::allocate_buffers() {
    check_cuda(cudaMalloc(&d_input_,  input_size()  * sizeof(float)), "cudaMalloc input");
    check_cuda(cudaMalloc(&d_output_, output_size() * sizeof(float)), "cudaMalloc output");
}

void SegmentationEngine::free_buffers() {
    if (d_input_)  { cudaFree(d_input_);  d_input_  = nullptr; }
    if (d_output_) { cudaFree(d_output_); d_output_ = nullptr; }
    if (stream_)   { cudaStreamDestroy(static_cast<cudaStream_t>(stream_)); stream_ = nullptr; }
}

void SegmentationEngine::infer(const std::vector<float>& input,
                               std::vector<float>& output_logits) {
    if (!loaded_) {
        throw std::runtime_error("SegmentationEngine not loaded");
    }
    if (input.size() != static_cast<size_t>(input_size())) {
        throw std::runtime_error("input size mismatch: got " + std::to_string(input.size())
                                 + " expected " + std::to_string(input_size()));
    }
    output_logits.resize(static_cast<size_t>(output_size()));

    auto stream = static_cast<cudaStream_t>(stream_);
    auto* context = static_cast<nvinfer1::IExecutionContext*>(trt_context_);

    // TRT 10.x API: setInputTensorAddress / setOutputTensorAddress + enqueueV3(stream)
    if (!context->setInputTensorAddress("input", d_input_)) {
        throw std::runtime_error("TensorRT setInputTensorAddress('input') failed");
    }
    if (!context->setOutputTensorAddress("logits", d_output_)) {
        throw std::runtime_error("TensorRT setOutputTensorAddress('logits') failed");
    }

    check_cuda(cudaMemcpyAsync(d_input_, input.data(),
                               input.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream),
               "cudaMemcpyAsync H2D input");

    if (!context->enqueueV3(stream)) {
        throw std::runtime_error("TensorRT enqueueV3 returned false");
    }

    check_cuda(cudaMemcpyAsync(output_logits.data(), d_output_,
                               output_logits.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream),
               "cudaMemcpyAsync D2H output");
    check_cuda(cudaStreamSynchronize(stream), "cudaStreamSynchronize");
}

}  // namespace bev_segmentation
