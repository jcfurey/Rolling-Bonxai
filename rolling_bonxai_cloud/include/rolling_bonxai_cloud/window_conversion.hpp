// SPDX-License-Identifier: MIT
#pragma once
#include <rolling_bonxai_msgs/msg/surface_map_window.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace RollingBonxai {
struct SurfacePoint;
sensor_msgs::msg::PointCloud2 cloudOfWindowPoints(const std::vector<SurfacePoint>& points,
                                                  const std_msgs::msg::Header& header);
// Validates metadata and the complete bounded payload before publishing any
// decoded geometry. Coordinates retain the sender's map frame.
sensor_msgs::msg::PointCloud2 cloudFromWindow(const rolling_bonxai_msgs::msg::SurfaceMapWindow& window,
                                              size_t max_points = 1000000,
                                              size_t max_payload_bytes = 8 * 1024 * 1024);
}  // namespace RollingBonxai
