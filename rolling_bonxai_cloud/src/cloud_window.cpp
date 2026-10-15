// SPDX-License-Identifier: MIT
#include "rolling_bonxai_cloud/cloud_window.hpp"

#include <zstd.h>
#include <zstd_errors.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace RollingBonxai {
namespace {
void validate(const SurfacePoint& p) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::isinf(p.intensity) ||
      std::isinf(p.reflectivity))
    throw std::invalid_argument("invalid surface point");
}
void check(size_t result) {
  if (ZSTD_isError(result)) throw std::runtime_error(std::string("zstd: ") + ZSTD_getErrorName(result));
}
}  // namespace

CloudWindow::CloudWindow(const std::array<double, 3>& center, const std::array<double, 3>& extent,
                         WindowLimits limits)
    : limits_(limits) {
  if (!limits.max_points || limits.max_points > UINT32_MAX / sizeof(SurfacePoint) ||
      !limits.max_payload_bytes)
    throw std::invalid_argument("invalid window budgets");
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(center[k]) || !std::isfinite(extent[k]) || extent[k] <= 0.0)
      throw std::invalid_argument("invalid window bounds");
    lower_[k] = center[k] - extent[k] / 2.;
    upper_[k] = center[k] + extent[k] / 2.;
    if (!std::isfinite(lower_[k]) || !std::isfinite(upper_[k]) || lower_[k] >= upper_[k])
      throw std::invalid_argument("window bounds overflow");
  }
}

bool CloudWindow::add(const SurfacePoint& point) {
  validate(point);
  const float p[3] = {point.x, point.y, point.z};
  for (int k = 0; k < 3; ++k)
    if (p[k] < lower_[k] || p[k] >= upper_[k]) return false;
  if (points_.size() >= limits_.max_points) {
    failed_ = true;
    throw std::runtime_error("window exceeds point budget; reduce its extent");
  }
  // Bound capacity as well as size; vector's geometric growth can overshoot.
  if (points_.size() == points_.capacity())
    points_.reserve(std::min(limits_.max_points, std::max<size_t>(256, points_.capacity() * 2)));
  points_.push_back(point);
  return true;
}

std::vector<uint8_t> CloudWindow::compress() const {
  if (failed_) throw std::runtime_error("window exceeded its point budget");
  std::vector<uint8_t> raw(points_.size() * sizeof(SurfacePoint));
  const size_t count = points_.size();
  for (size_t i = 0; i < count; ++i) {
    for (size_t channel = 0; channel < 5; ++channel) {
      uint32_t bits;
      std::memcpy(&bits, reinterpret_cast<const uint8_t*>(&points_[i]) + 4 * channel, 4);
      for (size_t byte = 0; byte < 4; ++byte)
        raw[byte * 5 * count + channel * count + i] = static_cast<uint8_t>(bits >> (8 * byte));
    }
  }
  std::vector<uint8_t> result(std::min(ZSTD_compressBound(raw.size()), limits_.max_payload_bytes));
  ZSTD_CCtx* context = ZSTD_createCCtx();
  if (!context) throw std::bad_alloc();
  try {
    check(ZSTD_CCtx_setParameter(context, ZSTD_c_compressionLevel, 1));
    check(ZSTD_CCtx_setParameter(context, ZSTD_c_checksumFlag, 1));
    const size_t n = ZSTD_compress2(context, result.data(), result.size(), raw.data(), raw.size());
    if (ZSTD_getErrorCode(n) == ZSTD_error_dstSize_tooSmall)
      throw std::runtime_error("window exceeds compressed payload budget; reduce its extent");
    check(n);
    result.resize(n);
  } catch (...) {
    ZSTD_freeCCtx(context);
    throw;
  }
  ZSTD_freeCCtx(context);
  return result;
}

std::vector<SurfacePoint> decompressWindow(const uint8_t* payload, size_t payload_bytes, uint32_t count,
                                           size_t max_points, size_t max_payload_bytes) {
  if (count > max_points || count > UINT32_MAX / sizeof(SurfacePoint) || payload_bytes > max_payload_bytes)
    throw std::runtime_error("window exceeds receiver budget");
  const size_t bytes = static_cast<size_t>(count) * sizeof(SurfacePoint);
  if (!payload || payload_bytes < 5 || ZSTD_getFrameContentSize(payload, payload_bytes) != bytes ||
      ZSTD_findFrameCompressedSize(payload, payload_bytes) != payload_bytes || !(payload[4] & 4))
    throw std::runtime_error("invalid window frame, size or checksum flag");
  std::vector<uint8_t> raw(bytes);
  const size_t n = ZSTD_decompress(raw.data(), bytes, payload, payload_bytes);
  check(n);
  if (n != bytes) throw std::runtime_error("window byte count mismatch");
  std::vector<SurfacePoint> result(count);
  for (size_t i = 0; i < count; ++i) {
    for (size_t channel = 0; channel < 5; ++channel) {
      uint32_t bits = 0;
      for (size_t byte = 0; byte < 4; ++byte)
        bits |= static_cast<uint32_t>(raw[byte * 5 * count + channel * count + i]) << (8 * byte);
      std::memcpy(reinterpret_cast<uint8_t*>(&result[i]) + 4 * channel, &bits, 4);
    }
  }
  for (const auto& p : result) validate(p);
  return result;
}
}  // namespace RollingBonxai
