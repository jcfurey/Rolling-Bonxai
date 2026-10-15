// SPDX-License-Identifier: MIT
#include "rolling_bonxai_cloud/window_conversion.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "rolling_bonxai_cloud/cloud_window.hpp"

namespace RollingBonxai {
sensor_msgs::msg::PointCloud2 cloudFromWindow(const rolling_bonxai_msgs::msg::SurfaceMapWindow& w,
                                              size_t max_points, size_t max_payload_bytes) {
  const auto& p = w.center_pose.position;
  const auto& q = w.center_pose.orientation;
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (w.schema_version != 1 || w.encoding != kSurfaceEncoding || w.header.frame_id.empty() ||
      w.source_id.empty() || w.map_id.empty() || w.center_keyframe < 0 ||
      static_cast<uint64_t>(w.center_keyframe) >= w.keyframe_count || w.header.stamp.nanosec >= 1000000000 ||
      w.source_stamp.nanosec >= 1000000000 || w.center_stamp.nanosec >= 1000000000 ||
      !std::isfinite(w.resolution) || w.resolution < 0 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !std::isfinite(norm2) || std::abs(norm2 - 1.) > 1e-3 ||
      !std::isfinite(w.extent.x) || !std::isfinite(w.extent.y) || !std::isfinite(w.extent.z) ||
      w.extent.x <= 0 || w.extent.y <= 0 || w.extent.z <= 0 ||
      static_cast<uint64_t>(w.point_count) * sizeof(SurfacePoint) != w.uncompressed_bytes)
    throw std::invalid_argument("invalid surface window metadata");
  const auto points = decompressWindow(w.data.empty() ? nullptr : &w.data[0], w.data.size(), w.point_count,
                                       max_points, max_payload_bytes);
  // A packet must obey its own declared half-open spatial bounds too.
  const double center[3] = {p.x, p.y, p.z}, extent[3] = {w.extent.x, w.extent.y, w.extent.z};
  for (int k = 0; k < 3; ++k) {
    const double lower = center[k] - extent[k] / 2., upper = center[k] + extent[k] / 2.;
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
      throw std::invalid_argument("window bounds overflow");
    for (const auto& point : points) {
      const float v[3] = {point.x, point.y, point.z};
      if (v[k] < lower || v[k] >= upper) throw std::invalid_argument("point outside declared window");
    }
  }
  return cloudOfWindowPoints(points, w.header);
}

sensor_msgs::msg::PointCloud2 cloudOfWindowPoints(const std::vector<SurfacePoint>& points,
                                                  const std_msgs::msg::Header& header) {
  if (points.size() > UINT32_MAX / sizeof(SurfacePoint)) throw std::invalid_argument("cloud size overflow");
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = points.size();
  const uint16_t one = 1;
  cloud.is_bigendian = *reinterpret_cast<const uint8_t*>(&one) != 1;
  cloud.is_dense = true;
  cloud.point_step = sizeof(SurfacePoint);
  cloud.row_step = cloud.point_step * cloud.width;
  for (const char* name : {"x", "y", "z", "intensity", "reflectivity"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = 4 * cloud.fields.size();
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step);
  if (!points.empty()) std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  for (const auto& point : points)
    if (!std::isfinite(point.intensity) || !std::isfinite(point.reflectivity)) cloud.is_dense = false;
  return cloud;
}
}  // namespace RollingBonxai
