// Phase 0 engine benchmark: BEVDet FP16 on the phase0 sample,
// warmup + N measured frames, writes benchmark JSON.
// Usage: bevdet_bench <configure_yaml> <output_json> [num_frames] [warmup]
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <cuda_runtime.h>

#include "bevdet.h"
#include "cpu_jpegdecoder.h"

static std::string gpuName() {
  cudaDeviceProp p{};
  cudaGetDeviceProperties(&p, 0);
  return std::string(p.name);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    printf("usage: %s <configure.yaml> <out.json> [frames=50] [warmup=5]\n", argv[0]);
    return 1;
  }
  const int frames = argc > 3 ? std::atoi(argv[3]) : 50;
  const int warmup = argc > 4 ? std::atoi(argv[4]) : 5;

  YAML::Node config = YAML::LoadFile(argv[1]);
  size_t img_N = config["N"].as<size_t>();
  int img_w = config["W"].as<int>();
  int img_h = config["H"].as<int>();
  std::string model_config = config["ModelConfig"].as<std::string>();
  std::string engine_file = config["EngineFile"].as<std::string>();
  std::string onnx_file = config["OnnxFile"] ? config["OnnxFile"].as<std::string>() : "";
  std::string precision = config["Precision"] ? config["Precision"].as<std::string>() : "fp16";
  YAML::Node camconfig = YAML::LoadFile(config["CamConfig"].as<std::string>());
  YAML::Node sample = config["sample"];

  std::vector<std::string> imgs_file, imgs_name;
  for (auto f : sample) {
    imgs_file.push_back(f.second.as<std::string>());
    imgs_name.push_back(f.first.as<std::string>());
  }

  camsData sampleData;
  sampleData.param = camParams(camconfig, img_N, imgs_name);

  BEVDet bevdet(model_config, img_N, sampleData.param.cams_intrin,
                sampleData.param.cams2ego_rot, sampleData.param.cams2ego_trans,
                onnx_file, engine_file, precision);

  std::vector<std::vector<char>> imgs_data;
  read_sample(imgs_file, imgs_data);
  uchar* imgs_dev = nullptr;
  CHECK_CUDA(cudaMalloc((void**)&imgs_dev, img_N * 3 * img_w * img_h * sizeof(uchar)));
  decode_cpu(imgs_data, imgs_dev, img_w, img_h);
  sampleData.imgs_dev = imgs_dev;

  std::vector<float> times;
  times.reserve(frames);
  std::vector<Box> ego_boxes;
  for (int i = 0; i < warmup + frames; ++i) {
    ego_boxes.clear();
    float t = 0.f;
    bevdet.DoInfer(sampleData, ego_boxes, t, i);
    if (i >= warmup) times.push_back(t);
    if (i == 0) {
      printf("first frame: %d objects, t=%.3f ms\n", (int)ego_boxes.size(), t);
    }
  }

  std::sort(times.begin(), times.end());
  auto pct = [&](double p) {
    size_t idx = std::min<size_t>((size_t)(p * times.size()), times.size() - 1);
    return times[idx];
  };
  double mean = 0;
  for (float t : times) mean += t;
  mean /= times.size();

  long total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count(); // placeholder unused

  std::ofstream out(argv[2]);
  out << "{\n"
      << "  \"device\": \"" << gpuName() << "\",\n"
      << "  \"gpu_arch\": \"sm_87\",\n"
      << "  \"power_mode\": \"MODE_40W(nvpmodel=3)\",\n"
      << "  \"jetpack_l4t\": \"R36.4.4\",\n"
      << "  \"cuda\": \"12.6.11\",\n"
      << "  \"tensorrt\": \"10.3.0\",\n"
      << "  \"model\": \"bevdet_one_lt_d\",\n"
      << "  \"precision\": \"" << precision << "\",\n"
      << "  \"input_resolution\": \"6x256x704 (image 900x400 int32 carriers)\",\n"
      << "  \"bev_resolution\": \"128x128 @0.8m\",\n"
      << "  \"batch\": 1,\n"
      << "  \"warmup_frames\": " << warmup << ",\n"
      << "  \"sample_count\": " << frames << ",\n"
      << "  \"mean_ms\": " << mean << ",\n"
      << "  \"median_ms\": " << pct(0.5) << ",\n"
      << "  \"p95_ms\": " << pct(0.95) << ",\n"
      << "  \"p99_ms\": " << pct(0.99) << "\n"
      << "}\n";
  out.close();
  printf("benchmark written: %s (mean=%.3f ms, median=%.3f, p95=%.3f, p99=%.3f)\n",
         argv[2], mean, pct(0.5), pct(0.95), pct(0.99));
  return 0;
}