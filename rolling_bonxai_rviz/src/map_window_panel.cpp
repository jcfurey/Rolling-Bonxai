// SPDX-License-Identifier: MIT
#include "rolling_bonxai_rviz/map_window_panel.hpp"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <limits>
#include <mutex>
#include <optional>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace RollingBonxai {
struct MapWindowPanel::Mailbox {
  std::mutex mutex;
  uint64_t generation = 0;
  std::optional<rolling_bonxai_msgs::msg::SurfaceWindowState> latest;
};
MapWindowPanel::MapWindowPanel(QWidget* parent)
    : rviz_common::Panel(parent), mailbox_(std::make_shared<Mailbox>()) {
  auto layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Map namespace", this));
  namespace_ = new QLineEdit("/", this);
  namespace_->setObjectName("window_namespace");
  layout->addWidget(namespace_);
  live_ = new QCheckBox("Follow live", this);
  live_->setChecked(true);
  live_->setObjectName("window_follow_live");
  layout->addWidget(live_);
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setObjectName("window_pose_slider");
  slider_->setRange(0, 0);
  slider_->setEnabled(false);
  layout->addWidget(slider_);
  auto row = new QHBoxLayout;
  auto previous = new QPushButton("Previous", this);
  auto next = new QPushButton("Next", this);
  row->addWidget(previous);
  row->addWidget(next);
  layout->addLayout(row);
  pose_ = new QLabel("Waiting for map poses", this);
  pose_->setObjectName("window_pose_label");
  layout->addWidget(pose_);
  status_ = new QLabel("Current map cropped at the selected pose", this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();
  debounce_ = new QTimer(this);
  debounce_->setSingleShot(true);
  debounce_->setInterval(150);
  refresh_ = new QTimer(this);
  refresh_->setInterval(100);
  refresh_->start();
  connect(refresh_, &QTimer::timeout, this, &MapWindowPanel::refresh);
  connect(debounce_, &QTimer::timeout, this, &MapWindowPanel::publishSelection);
  connect(namespace_, &QLineEdit::editingFinished, this, [this] {
    connectTopics();
    Q_EMIT configChanged();
  });
  connect(slider_, &QSlider::valueChanged, this, [this](int index) {
    QSignalBlocker guard(live_);
    live_->setChecked(false);
    desired_ = index;
    pending_ = true;
    debounce_->start();
    pose_->setText(QString("Requesting pose %1").arg(index));
  });
  connect(live_, &QCheckBox::toggled, this, [this](bool live) {
    debounce_->stop();
    desired_ = live ? -1 : slider_->value();
    pending_ = true;
    publishSelection();
  });
  connect(previous, &QPushButton::clicked, this, [this] {
    if (slider_->isEnabled()) slider_->setValue(slider_->value() - 1);
  });
  connect(next, &QPushButton::clicked, this, [this] {
    if (slider_->isEnabled()) slider_->setValue(slider_->value() + 1);
  });
}
MapWindowPanel::~MapWindowPanel() { state_sub_.reset(); }
void MapWindowPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (abstraction) initializeNode(abstraction->get_raw_node());
}
void MapWindowPanel::initializeNode(rclcpp::Node::SharedPtr node) {
  node_ = std::move(node);
  connectTopics();
}
void MapWindowPanel::connectTopics() {
  if (!node_) return;
  state_sub_.reset();
  select_pub_.reset();
  pending_ = false;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    generation = ++mailbox_->generation;
    mailbox_->latest.reset();
  }
  QString prefix = namespace_->text().trimmed();
  if (!prefix.startsWith('/')) prefix.prepend('/');
  while (prefix.endsWith('/')) prefix.chop(1);
  try {
    select_pub_ = node_->create_publisher<std_msgs::msg::Int64>((prefix + "/map_window/select").toStdString(),
                                                                rclcpp::QoS(1).reliable());
    auto mailbox = mailbox_;
    state_sub_ = node_->create_subscription<rolling_bonxai_msgs::msg::SurfaceWindowState>(
        (prefix + "/map_window/state").toStdString(), rclcpp::QoS(1).transient_local().reliable(),
        [mailbox, generation](rolling_bonxai_msgs::msg::SurfaceWindowState::ConstSharedPtr state) {
          std::lock_guard<std::mutex> lock(mailbox->mutex);
          if (mailbox->generation == generation) mailbox->latest = *state;
        });
    status_->setText("Waiting for local-map state");
  } catch (const std::exception& error) {
    status_->setText(QString::fromUtf8(error.what()));
  }
}
void MapWindowPanel::publishSelection() {
  if (!select_pub_) return;
  std_msgs::msg::Int64 selection;
  selection.data = desired_;
  select_pub_->publish(selection);
}
void MapWindowPanel::refresh() {
  std::optional<rolling_bonxai_msgs::msg::SurfaceWindowState> state;
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    state.swap(mailbox_->latest);
  }
  if (!state) return;
  if (state->keyframe_count > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
    status_->setText("Pose count exceeds slider range");
    return;
  }
  QSignalBlocker slider_guard(slider_), live_guard(live_);
  slider_->setEnabled(state->keyframe_count > 0);
  slider_->setMaximum(state->keyframe_count ? state->keyframe_count - 1 : 0);
  if (pending_ && ((desired_ < 0 && state->following_live) ||
                   (!state->following_live && desired_ == state->selected_keyframe) ||
                   (desired_ >= 0 && static_cast<uint64_t>(desired_) >= state->keyframe_count)))
    pending_ = false;
  if (!pending_ && !slider_->isSliderDown()) {
    live_->setChecked(state->following_live);
    slider_->setValue(std::max<int64_t>(0, state->selected_keyframe));
  }
  if (pending_) return;
  pose_->setText(QString("Pose %1 / %2   t=%3.%4")
                     .arg(state->selected_keyframe)
                     .arg(state->keyframe_count ? state->keyframe_count - 1 : 0)
                     .arg(state->center_stamp.sec)
                     .arg(state->center_stamp.nanosec, 9, 10, QChar('0')));
  status_->setText(QString::fromStdString(state->status) +
                   QString("\n%1 points; %2 KiB compressed")
                       .arg(state->point_count)
                       .arg(state->compressed_bytes / 1024.0, 0, 'f', 1));
}
void MapWindowPanel::load(const rviz_common::Config& config) {
  Panel::load(config);
  QString value;
  if (config.mapGetString("Namespace", &value)) namespace_->setText(value);
  connectTopics();
}
void MapWindowPanel::save(rviz_common::Config config) const {
  Panel::save(config);
  config.mapSetValue("Namespace", namespace_->text());
}
}  // namespace RollingBonxai
PLUGINLIB_EXPORT_CLASS(RollingBonxai::MapWindowPanel, rviz_common::Panel)
