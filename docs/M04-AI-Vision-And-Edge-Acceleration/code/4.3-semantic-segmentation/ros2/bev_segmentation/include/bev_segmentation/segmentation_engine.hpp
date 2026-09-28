// include/bev_segmentation/segmentation_engine.hpp
//
// TensorRT 10.x 封装: 加载 .engine, 静态 [1,3,512,1024] -> [1,19,128,256]
//
// 设计要点:
//  - STATIC shape: 只分配一个 [1,3,512,1024] GPU buffer; 不暴露 batch/动态 shape API
//  - TRT 10.3 API: 使用 V2 builder/parser 风格但通过 runtime 加载已序列化 .engine
//  - 显式 stream, 显式 cudaMemcpy (host <-> device)
//  - 所有 TensorRT/CUDA 调用 fail-fast; throw std::runtime_error

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Forward declarations - 避免在头文件内引入 TensorRT 头文件 (链接膨胀)
namespace nvinfer1 { class IRuntime; class ICudaEngine; class IExecutionContext; }

namespace bev_segmentation {

class SegmentationEngine {
public:
    SegmentationEngine();
    ~SegmentationEngine();

    // 禁止 copy / move (持有 CUDA / TensorRT 资源)
    SegmentationEngine(const SegmentationEngine&) = delete;
    SegmentationEngine& operator=(const SegmentationEngine&) = delete;

    // 加载 FP16 engine. 成功后立即分配 device memory.
    // 失败抛 std::runtime_error.
    void load(const std::string& engine_path);

    // 推理: input 长度必须为 1*3*512*1024 (RGB float32).
    // output_logits 长度 1*19*128*256 (host buffer 预分配).
    void infer(const std::vector<float>& input,
               std::vector<float>& output_logits);

    // 元信息
    bool is_loaded() const { return loaded_; }
    int input_size() const  { return 1 * 3 * 512 * 1024; }
    int output_size() const { return 1 * 19 * 128 * 256; }
    int logit_h() const { return 128; }
    int logit_w() const { return 256; }
    int num_classes() const { return 19; }

private:
    void allocate_buffers();
    void free_buffers();

    bool loaded_ = false;
    void* trt_runtime_   = nullptr;  // nvinfer1::IRuntime*
    void* trt_engine_    = nullptr;  // nvinfer1::ICudaEngine*
    void* trt_context_   = nullptr;  // nvinfer1::IExecutionContext*
    void* stream_        = nullptr;  // cudaStream_t

    void* d_input_       = nullptr;
    void* d_output_      = nullptr;
};

}  // namespace bev_segmentation
