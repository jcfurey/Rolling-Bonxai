// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <QApplication>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <chrono>
#include <functional>
#include <thread>

#include "rolling_bonxai_rviz/map_window_panel.hpp"

class WindowPanelTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    static int argc = 1;
    static char arg[] = "window_panel_test";
    static char* argv[] = {arg, nullptr};
    app = std::make_unique<QApplication>(argc, argv);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
    app.reset();
  }
  void SetUp() override {
    node = std::make_shared<rclcpp::Node>("window_panel_test");
    executor.add_node(node);
    state_pub = node->create_publisher<rolling_bonxai_msgs::msg::SurfaceWindowState>(
        "/panel_test/map_window/state", rclcpp::QoS(1).transient_local());
    selection_sub = node->create_subscription<std_msgs::msg::Int64>(
        "/panel_test/map_window/select", 10,
        [this](std_msgs::msg::Int64::ConstSharedPtr msg) { selection = msg->data; });
    panel = std::make_unique<RollingBonxai::MapWindowPanel>();
    panel->findChild<QLineEdit*>("window_namespace")->setText("/panel_test");
    panel->initializeNode(node);
    slider = panel->findChild<QSlider*>("window_pose_slider");
    live = panel->findChild<QCheckBox*>("window_follow_live");
    ASSERT_TRUE(wait([this] { return state_pub->get_subscription_count() > 0; }));
  }
  void TearDown() override {
    panel.reset();
    executor.remove_node(node);
    selection_sub.reset();
    state_pub.reset();
    node.reset();
  }
  bool wait(const std::function<bool()>& predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      app->processEvents();
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
  }
  void state(int64_t index, bool following, uint64_t count = 5) {
    rolling_bonxai_msgs::msg::SurfaceWindowState s;
    s.keyframe_count = count;
    s.selected_keyframe = index;
    s.following_live = following;
    s.available = true;
    s.status = "ready";
    state_pub->publish(s);
  }
  static std::unique_ptr<QApplication> app;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<RollingBonxai::MapWindowPanel> panel;
  rclcpp::Publisher<rolling_bonxai_msgs::msg::SurfaceWindowState>::SharedPtr state_pub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr selection_sub;
  QSlider* slider;
  QCheckBox* live;
  int64_t selection = -99;
};
std::unique_ptr<QApplication> WindowPanelTest::app;
TEST_F(WindowPanelTest, SliderPublishesPoseAndLiveReturnsToLatest) {
  state(4, true);
  ASSERT_TRUE(wait([this] { return slider->maximum() == 4; }));
  EXPECT_TRUE(live->isChecked());
  slider->setValue(1);
  ASSERT_TRUE(wait([this] { return selection == 1; }));
  EXPECT_FALSE(live->isChecked());
  state(1, false);
  ASSERT_TRUE(
      wait([this] { return panel->findChild<QLabel*>("window_pose_label")->text().startsWith("Pose 1"); }));
  live->setChecked(true);
  ASSERT_TRUE(wait([this] { return selection == -1; }));
  state(4, true);
  ASSERT_TRUE(wait([this] { return slider->value() == 4 && live->isChecked(); }));
}
TEST_F(WindowPanelTest, NamespaceChangeDiscardsOldState) {
  state(4, true);
  ASSERT_TRUE(wait([this] { return slider->maximum() == 4; }));
  auto other = node->create_publisher<rolling_bonxai_msgs::msg::SurfaceWindowState>(
      "/other/map_window/state", rclcpp::QoS(1).transient_local());
  auto field = panel->findChild<QLineEdit*>("window_namespace");
  field->setText("/other");
  QMetaObject::invokeMethod(field, "editingFinished", Qt::DirectConnection);
  ASSERT_TRUE(wait([&] { return other->get_subscription_count() > 0; }));
  state(99, true, 100);
  rolling_bonxai_msgs::msg::SurfaceWindowState s;
  s.keyframe_count = 3;
  s.selected_keyframe = 2;
  s.following_live = true;
  other->publish(s);
  ASSERT_TRUE(wait([this] { return slider->maximum() == 2; }));
  EXPECT_EQ(slider->value(), 2);
}
