// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace RollingBonxai {

// Logical order is x/y/z/intensity/reflectivity, float32 little endian.
// The compressed input is four byte planes, each containing five channel
// arrays in that order. This rearranges bytes without quantizing any value.
// Geometry is in the packet's named parent frame; the box is axis-aligned
// in that frame. No point is resampled, averaged or moved by this exporter.
struct SurfacePoint {
  float x, y, z;
  float intensity = std::numeric_limits<float>::quiet_NaN();
  float reflectivity = std::numeric_limits<float>::quiet_NaN();
};
static_assert(sizeof(SurfacePoint) == 20);
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
inline constexpr const char* kSurfaceEncoding = "xyz_intensity_reflectivity_f32le_planes_zstd_v1";

struct WindowLimits {
  size_t max_points = 1000000;
  size_t max_payload_bytes = 8 * 1024 * 1024;
};

// One bounded snapshot. Exceeding a budget rejects the snapshot rather than
// silently removing geometry or increasing its voxel size. Owned by one worker.
class CloudWindow {
 public:
  CloudWindow(const std::array<double, 3>& center, const std::array<double, 3>& extent,
              WindowLimits limits = {});
  bool add(const SurfacePoint& point);
  const std::vector<SurfacePoint>& points() const { return points_; }
  std::vector<uint8_t> compress() const;

 private:
  std::array<double, 3> lower_, upper_;
  WindowLimits limits_;
  std::vector<SurfacePoint> points_;
  bool failed_ = false;
};

// Validate declared size and zstd frame size before allocating, and require a
// checksummed frame. max_points is the receiving platform's own budget.
std::vector<SurfacePoint> decompressWindow(const uint8_t* payload, size_t payload_bytes, uint32_t point_count,
                                           size_t max_points = 1000000,
                                           size_t max_payload_bytes = 8 * 1024 * 1024);
inline std::vector<SurfacePoint> decompressWindow(const std::vector<uint8_t>& payload, uint32_t point_count,
                                                  size_t max_points = 1000000,
                                                  size_t max_payload_bytes = 8 * 1024 * 1024) {
  return decompressWindow(payload.data(), payload.size(), point_count, max_points, max_payload_bytes);
}

}  // namespace RollingBonxai
