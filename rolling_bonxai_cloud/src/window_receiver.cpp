// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "rolling_bonxai_cloud/window_conversion.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("surface_window_receiver");
  const int64_t max_points = node->declare_parameter<int64_t>("max_points", 1000000);
  const int64_t max_bytes = node->declare_parameter<int64_t>("max_payload_bytes", 8 * 1024 * 1024);
  if (max_points <= 0 || max_points > UINT32_MAX / 20 || max_bytes <= 0)
    throw std::invalid_argument("invalid receiver budgets");
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_window/received_cloud", rclcpp::QoS(1).transient_local().reliable());
  auto subscription = node->create_subscription<rolling_bonxai_msgs::msg::SurfaceMapWindow>(
      "map_window/packet", rclcpp::QoS(1).transient_local().reliable(),
      [node, publisher, max_points,
       max_bytes](rolling_bonxai_msgs::msg::SurfaceMapWindow::ConstSharedPtr window) {
        try {
          publisher->publish(RollingBonxai::cloudFromWindow(*window, max_points, max_bytes));
        } catch (const std::exception& e) {
          RCLCPP_WARN(node->get_logger(), "Rejected map window: %s", e.what());
        }
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
