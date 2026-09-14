#include "bev_preprocessor/gpu_image_stager.hpp"

#include <cstring>
#include <stdexcept>

namespace bev::preprocessor
{

GpuImageStager::GpuImageStager(std::size_t n, std::size_t h, std::size_t w)
: n_(n), h_(h), w_(w)
{
  const std::size_t total = bytes();
  if (cudaMalloc(&d_buf_, total) != cudaSuccess) {
    throw std::runtime_error("GpuImageStager: cudaMalloc failed");
  }
  if (cudaMallocHost(&h_staging_, total) != cudaSuccess) {
    throw std::runtime_error("GpuImageStager: cudaMallocHost (pinned) failed");
  }
}

GpuImageStager::~GpuImageStager()
{
  if (d_buf_) cudaFree(d_buf_);
  if (h_staging_) cudaFreeHost(h_staging_);
}

cudaError_t GpuImageStager::stage(
  const std::vector<const std::uint8_t *> & imgs, cudaStream_t stream)
{
  if (imgs.size() != n_) return cudaErrorInvalidValue;
  const std::size_t plane = h_ * w_;
  const std::size_t img_bytes = plane * ch_;
  // planar BGR: channel order B,G,R (matches engine Preprocess kernel)
  for (std::size_t i = 0; i < n_; ++i) {
    const std::uint8_t * src = imgs.at(i);
    if (!src) return cudaErrorInvalidValue;
    std::uint8_t * dst = h_staging_ + i * img_bytes;
    for (std::size_t c = 0; c < ch_; ++c) {
      const std::uint8_t * src_plane = src + c;
      std::uint8_t * dst_plane = dst + c * plane;
      for (std::size_t row = 0; row < h_; ++row) {
        const std::uint8_t * src_row = src_plane + row * w_ * ch_;
        std::uint8_t * dst_row = dst_plane + row * w_;
        for (std::size_t col = 0; col < w_; ++col) {
          dst_row[col] = src_row[col * ch_];
        }
      }
    }
  }
  return cudaMemcpyAsync(d_buf_, h_staging_, bytes(), cudaMemcpyHostToDevice, stream);
}

bool GpuImageStager::verifyLayout(
  const std::vector<const std::uint8_t *> & imgs, cudaStream_t stream)
{
  if (stage(imgs, stream) != cudaSuccess) return false;
  const std::size_t total = bytes();
  auto * check = new std::uint8_t[total];
  cudaError_t err = cudaMemcpy(check, d_buf_, total, cudaMemcpyDeviceToHost);
  const std::size_t plane = h_ * w_;
  bool ok = (err == cudaSuccess);
  for (std::size_t i = 0; ok && i < n_; ++i) {
    const std::uint8_t * src = imgs.at(i);
    for (std::size_t c = 0; ok && c < ch_; ++c) {
      for (std::size_t row = 0; ok && row < h_; ++row) {
        for (std::size_t col = 0; col < w_; ++col) {
          const std::uint8_t expect = src[(row * w_ + col) * ch_ + c];
          const std::uint8_t got = check[i * plane * ch_ + c * plane + row * w_ + col];
          if (expect != got) ok = false;
        }
      }
    }
  }
  delete[] check;
  return ok;
}

}  // namespace bev::preprocessor