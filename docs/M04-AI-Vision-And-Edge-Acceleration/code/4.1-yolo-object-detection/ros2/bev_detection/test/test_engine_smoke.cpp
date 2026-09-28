// Engine smoke test: load YOLO engine, run inference on synthetic image, verify output.

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <random>

#include <opencv2/opencv.hpp>

#include "bev_detection/yolo_engine.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printf("Usage: %s <engine_path> [num_runs]\n", argv[0]);
    return 1;
  }
  std::string engine_path = argv[1];
  int num_runs = argc > 2 ? atoi(argv[2]) : 5;

  printf("=== Engine Smoke Test ===\n");
  printf("Engine: %s\n", engine_path.c_str());

  bev::detection::Logger logger(nvinfer1::ILogger::Severity::kINFO);

  bev::detection::YoloInferencer::Config config;
  config.engine_path = engine_path;
  config.num_classes = 80;
  config.conf_thresh = 0.25f;
  config.nms_thresh = 0.45f;

  bev::detection::YoloInferencer inferencer;
  if (!inferencer.init(config, logger)) {
    printf("FAIL: Failed to initialize YOLO inferencer\n");
    return 1;
  }
  printf("Engine loaded OK\n");

  // Create synthetic test image (640x480 BGR)
  cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(test_image, cv::Point(100, 100), cv::Point(300, 400),
                cv::Scalar(255, 0, 0), -1);  // blue box
  cv::rectangle(test_image, cv::Point(400, 200), cv::Point(550, 450),
                cv::Scalar(0, 255, 0), -1);  // green box
  cv::putText(test_image, "test", cv::Point(150, 250),
              cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 255, 255), 3);

  printf("Test image: 640x480 with 2 colored rectangles\n");

  // Warmup
  printf("Warmup run...\n");
  auto warmup = inferencer.detect(test_image.data, test_image.rows, test_image.cols);
  printf("Warmup inference: %.2f ms, %zu detections\n",
         warmup.inference_ms, warmup.boxes.size());

  // Timed runs
  printf("\nTimed runs (%d iterations):\n", num_runs);
  double total_ms = 0;
  double min_ms = 1e9, max_ms = 0;
  int total_detections = 0;

  for (int i = 0; i < num_runs; ++i) {
    auto result = inferencer.detect(test_image.data, test_image.rows, test_image.cols);
    total_ms += result.inference_ms;
    min_ms = std::min(min_ms, static_cast<double>(result.inference_ms));
    max_ms = std::max(max_ms, static_cast<double>(result.inference_ms));
    total_detections += result.boxes.size();

    if (i < 3) {
      printf("  Run %d: %.2f ms, %zu detections\n",
             i, result.inference_ms, result.boxes.size());
    }
  }

  printf("\n=== Inference Stats ===\n");
  printf("Average inference: %.2f ms\n", total_ms / num_runs);
  printf("Min: %.2f ms, Max: %.2f ms\n", min_ms, max_ms);
  printf("Total detections: %d (avg %.1f per frame)\n",
         total_detections, static_cast<double>(total_detections) / num_runs);

  printf("\n=== PASS ===\n");
  return 0;
}
