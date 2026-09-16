#pragma once
// GpuImageStager: convert N interleaved-BGR 8-bit images into the planar
// BGR GPU staging buffer expected by the BEVDet engine Preprocess plugin
// (uint8 [N,3,H,W] planar, byte-compatible with the engine's int32 carrier
// tensor [N,3,H,W/4]).
//
// Buffers are allocated ONCE and reused across frames (no per-frame
// cudaMalloc/cudaFree). Host staging memory is pinned.
#include <cstddef>
#include <cstdint>
#include <vector>

#include <cuda_runtime.h>

namespace bev::preprocessor
{

class GpuImageStager
{
public:
  GpuImageStager(std::size_t n, std::size_t h, std::size_t w);
  ~GpuImageStager();

  GpuImageStager(const GpuImageStager &) = delete;
  GpuImageStager & operator=(const GpuImageStager &) = delete;

  /// imgs: n pointers to interleaved BGR row-major uint8 buffers (h*w*3 each).
  /// Uploads into the preallocated device buffer on `stream`.
  /// Returns cudaError_t (cudaSuccess on success).
  cudaError_t stage(const std::vector<const std::uint8_t *> & imgs, cudaStream_t stream);

  /// Device buffer ready to feed the engine (freeze until next stage()).
  std::uint8_t * devicePtr() const { return d_buf_; }
  std::size_t bytes() const { return n_ * h_ * w_ * ch_; }

  std::size_t n() const { return n_; }
  std::size_t h() const { return h_; }
  std::size_t w() const { return w_; }

  /// Test helper: download and verify layout equality.
  bool verifyLayout(const std::vector<const std::uint8_t *> & imgs, cudaStream_t stream);

private:
  std::size_t n_, h_, w_;
  static constexpr int ch_ = 3;
  std::uint8_t * d_buf_{nullptr};
  std::uint8_t * h_staging_{nullptr};  // pinned
};

}  // namespace bev::preprocessor