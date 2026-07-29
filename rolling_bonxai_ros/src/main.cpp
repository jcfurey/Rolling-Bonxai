#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rolling_bonxai_ros/rolling_bonxai_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  {
    // Scoped so the node — and with it the flush of every dirty chunk in its
    // destructor — runs before rclcpp::shutdown() tears the context down.
    auto node = std::make_shared<RollingBonxai::RollingBonxaiServer>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
