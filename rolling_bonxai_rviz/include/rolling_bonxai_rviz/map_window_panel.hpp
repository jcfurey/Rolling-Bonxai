// SPDX-License-Identifier: MIT
#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rolling_bonxai_msgs/msg/surface_window_state.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/int64.hpp>

class QLabel;
class QSlider;
class QCheckBox;
class QLineEdit;
class QTimer;
namespace RollingBonxai {
class MapWindowPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit MapWindowPanel(QWidget* parent = nullptr);
  ~MapWindowPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;
  // Also allows a headless test to exercise the real ROS/Qt controls.
  void initializeNode(rclcpp::Node::SharedPtr node);

 private:
  void connectTopics();
  void refresh();
  void publishSelection();
  struct Mailbox;
  std::shared_ptr<Mailbox> mailbox_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr select_pub_;
  rclcpp::Subscription<rolling_bonxai_msgs::msg::SurfaceWindowState>::SharedPtr state_sub_;
  QLineEdit* namespace_;
  QSlider* slider_;
  QCheckBox* live_;
  QLabel *pose_, *status_;
  QTimer *refresh_, *debounce_;
  int64_t desired_ = -1;
  bool pending_ = false;
};
}  // namespace RollingBonxai
