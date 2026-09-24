// test/parity_runner.cpp
//
// Offline PyTorch <-> TensorRT parity helper for M4.3.
//
// Why this exists as a separate binary: on this machine torch is only
// importable from the conda py310 environment while the `tensorrt` Python
// bindings are only importable from the system interpreter. The two cannot be
// loaded into one process, so the comparison is split in two halves and joined
// over raw files:
//
//   conda py310  : preprocess -> input.bin, PyTorch -> pytorch_logits.bin
//   this runner  : same input.bin -> the REAL production engine -> trt_logits.bin
//   conda py310  : verify_segformer_parity.py --compare, over the two buffers
//
// The point is that the TensorRT half runs the same SegmentationEngine the ROS
// node runs, so a green parity result actually covers the shipped code path
// rather than a reimplementation of it.

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "bev_segmentation/segmentation_engine.hpp"

namespace {

void usage(const char* argv0) {
    std::fprintf(stderr,
        "usage: %s --engine <file.engine> --input <input.bin> --output <trt_logits.bin>\n"
        "\n"
        "  input.bin       raw float32, 1*3*512*1024  (NCHW, RGB, letterboxed + ImageNet-normalised)\n"
        "  trt_logits.bin  raw float32, 1*19*128*256  (written)\n",
        argv0);
}

std::vector<float> read_floats(const std::string& path, size_t expected) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open for read: " + path);

    in.seekg(0, std::ios::end);
    const std::streamoff bytes = in.tellg();
    in.seekg(0, std::ios::beg);
    if (bytes < 0) throw std::runtime_error("cannot size file: " + path);

    const size_t got = static_cast<size_t>(bytes) / sizeof(float);
    if (got != expected) {
        throw std::runtime_error(
            "size mismatch in " + path + ": expected " + std::to_string(expected) +
            " float32 values, got " + std::to_string(got));
    }

    std::vector<float> v(expected);
    in.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    if (!in) throw std::runtime_error("short read from " + path);
    return v;
}

void write_floats(const std::string& path, const std::vector<float>& v) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot open for write: " + path);
    out.write(reinterpret_cast<const char*>(v.data()),
              static_cast<std::streamsize>(v.size() * sizeof(float)));
    if (!out) throw std::runtime_error("short write to " + path);
}

}  // namespace

int main(int argc, char** argv) {
    std::string engine_path, input_path, output_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--engine" && i + 1 < argc)      engine_path = argv[++i];
        else if (a == "--input" && i + 1 < argc)  input_path  = argv[++i];
        else if (a == "--output" && i + 1 < argc) output_path = argv[++i];
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else { std::fprintf(stderr, "unexpected argument: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }

    if (engine_path.empty() || input_path.empty() || output_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    try {
        bev_segmentation::SegmentationEngine engine;
        engine.load(engine_path);
        std::printf("[parity_runner] engine : %s\n", engine_path.c_str());
        std::printf("[parity_runner] shape  : in=%d out=%d logits=%dx%d classes=%d\n",
                    engine.input_size(), engine.output_size(),
                    engine.logit_h(), engine.logit_w(), engine.num_classes());

        const std::vector<float> input =
            read_floats(input_path, static_cast<size_t>(engine.input_size()));
        std::printf("[parity_runner] input  : %zu floats\n", input.size());

        std::vector<float> logits(static_cast<size_t>(engine.output_size()));
        engine.infer(input, logits);

        write_floats(output_path, logits);
        std::printf("[parity_runner] output : %zu floats -> %s\n",
                    logits.size(), output_path.c_str());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[parity_runner] FAILED: %s\n", e.what());
        return 1;
    }
}