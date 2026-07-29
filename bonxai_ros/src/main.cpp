#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bonxai_ros/bonxai_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  {
    // Scoped so the node is destroyed before rclcpp::shutdown() tears the
    // context down.
    auto node = std::make_shared<Bonxai::BonxaiServer>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
