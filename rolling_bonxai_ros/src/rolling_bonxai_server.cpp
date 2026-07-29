#include "rolling_bonxai_ros/rolling_bonxai_server.hpp"

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace
{
inline double bytesToMiB(std::size_t bytes)
{
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}
}

namespace RollingBonxai
{

RollingBonxaiServer::RollingBonxaiServer(
  const rclcpp::NodeOptions& options)
: rclcpp::Node("rolling_bonxai_server", options)
{
  loadParameters();
  initLogger();
  initTF();
  initMap();
  initSubscriber();
  initService();
  initStatsPublishers();
  initChunkVisualization();

  // use_sim_time is pre-declared by rclcpp. Report it, because almost every
  // "the map never updates" report against a simulated robot comes down to this
  // being false while /clock is the only time source in the graph.
  const bool use_sim_time = get_parameter("use_sim_time").as_bool();
  RCLCPP_INFO(
    get_logger(),
    "RollingBonxaiServer started (use_sim_time=%s, clock=%s)",
    use_sim_time ? "true" : "false",
    use_sim_time ? "/clock" : "system");

  if (use_sim_time) {
    RCLCPP_INFO(
      get_logger(),
      "Simulated time is in use: timers and chunk ageing follow /clock, and "
      "nothing will fire until /clock starts publishing.");
  }
}

RollingBonxaiServer::~RollingBonxaiServer()
{
  // Chunks only reach disk when the policy evicts them, so without a flush the
  // whole neighbourhood around the robot is lost on exit. ~RollingOccupancyMap
  // would do this anyway; doing it here orders the flush ahead of the rest of
  // the node's teardown, while the rclcpp::Logger the engine writes through is
  // still plainly alive. flushAllChunks() is idempotent, so the second call
  // from the engine's own destructor is a no-op.
  if (occupancy_map_) {
    occupancy_map_->flushAllChunks();
  }
}

void RollingBonxaiServer::loadParameters()
{
  server_params_.frame_id = declare_parameter<std::string>("frame_id", "map");
  server_params_.base_frame_id = declare_parameter<std::string>("base_frame_id", "base_footprint");
  server_params_.topic_in = declare_parameter<std::string>("topic_in", "/points");

  server_params_.cleanup_interval_sec =
    declare_parameter<double>("server.cleanup_interval_sec", 300.0);

  server_params_.tf_timeout_sec =
    declare_parameter<double>("server.tf_timeout_sec", 0.1);

  server_params_.enable_stats =
    declare_parameter<bool>("stats.enable_stats", true);

  server_params_.quick_stats =
    declare_parameter<bool>("stats.quick_stats", true);

  server_params_.publish_occupied_voxels =
    declare_parameter<bool>("stats.publish_occupied_voxels", true);

  server_params_.stats_publish_rate =
    declare_parameter<double>("stats.publish_rate", 1.0);

  server_params_.chunk_vis_publish_rate =
    declare_parameter<double>("stats.chunk_vis_publish_rate", 1.0);

  auto& mp = rolling_params_.map_params;

  mp.resolution = declare_parameter<double>("occupancy.resolution", 0.05);
  mp.occupancy_min_z = declare_parameter<double>("occupancy.occupancy_min_z", -100.0);
  mp.occupancy_max_z = declare_parameter<double>("occupancy.occupancy_max_z", 100.0);
  mp.occupancy_threshold = declare_parameter<double>("occupancy.occupancy_threshold", 0.5);

  mp.sensor_max_range = declare_parameter<double>("sensor_model.max_range", 30.0);
  mp.sensor_hit = declare_parameter<double>("sensor_model.hit", 0.7);
  mp.sensor_miss = declare_parameter<double>("sensor_model.miss", 0.4);
  mp.sensor_min = declare_parameter<double>("sensor_model.min", 0.12);
  mp.sensor_max = declare_parameter<double>("sensor_model.max", 0.97);

  rolling_params_.chunk_size =
    declare_parameter<double>("rolling.coordinate_system.chunk_size", 5.0);

  rolling_params_.hysteresis_ratio =
    declare_parameter<double>("rolling.transition_manager.hysteresis_ratio", 0.20);

  rolling_params_.loading_policy_name =
    declare_parameter<std::string>("rolling.loading_policy.name", "neighbourhood");

  rolling_params_.nb_radius =
    declare_parameter<int>("rolling.loading_policy.neighborhood.radius", 2);

  // planar.radius and temporal.radius were documented and shipped in the params
  // file but never declared, so both policies silently ran at the neighbourhood
  // radius no matter what the user configured.
  rolling_params_.planar_radius =
    declare_parameter<int>("rolling.loading_policy.planar.radius", 1);

  rolling_params_.planar_z_min =
    declare_parameter<int>("rolling.loading_policy.planar.z_min", 1);

  rolling_params_.planar_z_max =
    declare_parameter<int>("rolling.loading_policy.planar.z_max", 3);

  rolling_params_.planar_use_relative =
    declare_parameter<bool>("rolling.loading_policy.planar.use_relative", false);

  rolling_params_.temporal_radius =
    declare_parameter<int>("rolling.loading_policy.temporal.radius", 2);

  rolling_params_.temporal_age_weightage =
    declare_parameter<double>("rolling.loading_policy.temporal.age_weightage", 0.3);

  rolling_params_.temporal_max_age_s =
    declare_parameter<int>("rolling.loading_policy.temporal.max_age_s", 300);

  rolling_params_.full_save_folder_path =
    declare_parameter<std::string>("rolling.storage.full_save_folder_path", "");

  rolling_params_.asyncio.enable_io = declare_parameter<bool>("rolling.asyncio.enable_io", false);

  // declare_parameter<int> yields int64_t; clamp to >= 1 because a pool of zero
  // worker threads would leave every queued load or save permanently unserviced.
  rolling_params_.asyncio.num_load_threads = static_cast<size_t>(
    std::max<int64_t>(1, declare_parameter<int>("rolling.asyncio.num_load_threads", 3)));

  rolling_params_.asyncio.num_save_threads = static_cast<size_t>(
    std::max<int64_t>(1, declare_parameter<int>("rolling.asyncio.num_save_threads", 1)));

  // Enabling disk I/O without a destination silently writes nowhere, so treat
  // the empty path as "memory only" and say so.
  if (rolling_params_.asyncio.enable_io &&
      rolling_params_.full_save_folder_path.empty())
  {
    RCLCPP_ERROR(
      get_logger(),
      "rolling.asyncio.enable_io is true but rolling.storage.full_save_folder_path "
      "is empty. Disabling disk I/O — set a writable path to enable chunk persistence.");
    rolling_params_.asyncio.enable_io = false;
  }
}

void RollingBonxaiServer::initLogger()
{
  logger_ = std::make_shared<RosLogger>(get_logger());
}

void RollingBonxaiServer::initTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());

  // Give the buffer a ROS-time timer factory. Without it, any timeout-based
  // wait inside tf2 falls back to a steady-clock wait, which does not line up
  // with the sim-time stamps on incoming transforms.
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void RollingBonxaiServer::initMap()
{
  // Hand the node clock to the engine so chunk ageing is measured in the same
  // time domain the rest of the graph uses. Under use_sim_time this is /clock,
  // so temporal.max_age_s means simulated seconds rather than wall seconds.
  occupancy_map_ =
    std::make_unique<RollingBonxai::RollingOccupancyMap>(
      rolling_params_,
      logger_,
      [this]() -> int64_t { return this->now().nanoseconds(); });
}

void RollingBonxaiServer::initSubscriber()
{
  pointcloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
      server_params_.topic_in,
      rclcpp::SensorDataQoS(),
      std::bind(&RollingBonxaiServer::pointcloudCallback, this, std::placeholders::_1));
}

void RollingBonxaiServer::initService()
{
  clean_memory_srv_ =
    create_service<std_srvs::srv::Trigger>(
      "/rolling_bonxai/clean_memory",
      std::bind(
        &RollingBonxaiServer::cleanMemoryServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  if (server_params_.cleanup_interval_sec > 0.0) {
    cleanup_timer_ = createRosTimer(
      1.0 / server_params_.cleanup_interval_sec,
      std::bind(&RollingBonxaiServer::cleanupTimerCallback, this));
    RCLCPP_INFO(
      get_logger(), "Periodic memory cleanup enabled: every %.1fs",
      server_params_.cleanup_interval_sec);
  }
}

rclcpp::TimerBase::SharedPtr RollingBonxaiServer::createRosTimer(
  double rate_hz, std::function<void()> callback)
{
  // A non-positive rate yields an infinite or negative period. Both make
  // rclcpp throw, so clamp and report instead of taking the node down.
  if (!(rate_hz > 0.0)) {
    RCLCPP_WARN(
      get_logger(),
      "Requested timer rate %.3f Hz is not positive; falling back to 1.0 Hz",
      rate_hz);
    rate_hz = 1.0;
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));

  return rclcpp::create_timer(this, get_clock(), rclcpp::Duration(period), std::move(callback));
}

void RollingBonxaiServer::initStatsPublishers()
{
  if (!server_params_.enable_stats) {
    return;
  }

  if (server_params_.publish_occupied_voxels) {
    voxel_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/rolling_bonxai/occupied_voxels",
        rclcpp::QoS(1).transient_local());
  }

  chunk_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::ChunkStats>("/rolling_bonxai/chunk_stats", 10);

  quick_occ_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::QuickOccupancyStats>(
      
        "/rolling_bonxai/quick_occupancy_stats", 10);

  stats_timer_ = createRosTimer(
    server_params_.stats_publish_rate,
    std::bind(&RollingBonxaiServer::publishStats, this));
}

void RollingBonxaiServer::initChunkVisualization()
{
  chunk_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/rolling_bonxai/chunk_markers", 1);

  transition_state_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::RollingState>("/rolling_bonxai/transition_state", 10);

  chunk_vis_timer_ = createRosTimer(
    server_params_.chunk_vis_publish_rate,
    std::bind(&RollingBonxaiServer::publishChunkVisualization, this));
}

void RollingBonxaiServer::cleanMemoryServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!occupancy_map_->isStatsStable()) {
    response->success = false;
    response->message = "Memory cleanup skipped: map not in stats-safe state";
    return;
  }

  occupancy_map_->releaseUnusedMemory();
  occupancy_map_->shrinkToFit();

  response->success = true;
  response->message = "RollingBonxai memory cleanup completed";
}

void RollingBonxaiServer::cleanupTimerCallback()
{
  if (!occupancy_map_ || !occupancy_map_->isStatsStable()) {
    // Chunk I/O is in flight; try again next cycle rather than touching grids
    // that a worker thread may be serialising.
    return;
  }

  const size_t before = occupancy_map_->getMemoryUsage();
  occupancy_map_->releaseUnusedMemory();
  occupancy_map_->shrinkToFit();
  const size_t after = occupancy_map_->getMemoryUsage();

  RCLCPP_INFO(
    get_logger(), "Periodic cleanup: %.1f MiB -> %.1f MiB (freed %.1f MiB)",
    bytesToMiB(before), bytesToMiB(after),
    bytesToMiB(before) - bytesToMiB(after));
}

void RollingBonxaiServer::publishStats()
{
  const auto now_time = now();

  if (voxel_pub_) {
    std::vector<Eigen::Vector3d> points;
    occupancy_map_->getOccupiedVoxels(points);

    if (!points.empty()) {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      cloud.reserve(points.size());

      for (const auto& p : points) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud.push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = server_params_.frame_id;
      msg.header.stamp = now_time;
      voxel_pub_->publish(msg);
    }
  }

  rolling_bonxai_msgs::msg::ChunkStats cs;
  cs.header.stamp = now_time;
  cs.header.frame_id = server_params_.frame_id;
  cs.active_chunk_count = occupancy_map_->getActiveChunkCount();
  cs.dirty_chunk_count = occupancy_map_->getDirtyChunkCount();
  cs.clean_chunk_count = occupancy_map_->getCleanChunkCount();
  chunk_stats_pub_->publish(cs);

  // stats.quick_stats was declared but ignored, so the expensive full-cell
  // iteration could never actually be requested. Honour it now.
  const auto qs = occupancy_map_->getStats(!server_params_.quick_stats);
  rolling_bonxai_msgs::msg::QuickOccupancyStats qs_msg;
  qs_msg.header.stamp = now_time;
  qs_msg.header.frame_id = server_params_.frame_id;
  qs_msg.total_memory_mib = bytesToMiB(qs.total_memory_bytes);
  qs_msg.grid_memory_mib = bytesToMiB(qs.grid_memory_bytes);
  qs_msg.buffer_memory_mib = bytesToMiB(qs.buffer_memory_bytes);
  qs_msg.total_active_cells = qs.total_active_cells;
  qs_msg.total_insertions = qs.total_insertions;
  qs_msg.hit_points_added = qs.hit_points_added;
  qs_msg.miss_points_added = qs.miss_points_added;
  qs_msg.has_data = qs.has_data;
  quick_occ_stats_pub_->publish(qs_msg);
}

void RollingBonxaiServer::publishChunkVisualization()
{
  const auto now_time = now();

  const auto states = occupancy_map_->getChunkStates();
  const auto pending = occupancy_map_->getPendingChunks();

  visualization_msgs::msg::MarkerArray arr;
  const double size = rolling_params_.chunk_size;
  int id = 0;

  for (const auto& [pos, dirty] : states) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;

    if (dirty) {
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
    } else {
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
    }
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  for (const auto& pos : pending) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  // Markers persist in RViz until explicitly removed. Once the rolling window
  // shrinks, every id we published last cycle but not this one would otherwise
  // stay on screen forever as a ghost chunk.
  for (int stale_id = id; stale_id < last_marker_count_; ++stale_id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = stale_id;
    m.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(m);
  }
  last_marker_count_ = id;

  chunk_marker_pub_->publish(arr);

  rolling_bonxai_msgs::msg::RollingState ts;
  ts.header.stamp = now_time;
  ts.header.frame_id = server_params_.frame_id;
  ts.state = occupancy_map_->getTransitionState();
  transition_state_pub_->publish(ts);
}

void RollingBonxaiServer::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // An all-zero stamp means the publisher never filled the header. Asking tf2
  // for transform-at-time-zero is interpreted as "the latest available", which
  // is the only sensible reading; looking up an actual epoch-zero timestamp
  // would fail on every cloud.
  const rclcpp::Time stamp(msg->header.stamp);
  const tf2::TimePoint lookup_time =
    (stamp.nanoseconds() == 0)
      ? tf2::TimePointZero
      : tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds()));

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      server_params_.frame_id,
      msg->header.frame_id,
      lookup_time,
      tf2::durationFromSec(server_params_.tf_timeout_sec));
  } catch (const tf2::TransformException& ex) {
    // Throttled: on a mis-configured frame this fires at full sensor rate.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF lookup %s -> %s failed: %s",
      msg->header.frame_id.c_str(), server_params_.frame_id.c_str(), ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  Eigen::Vector3d origin(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z);

  Eigen::Vector3d linvel(0.0, 0.0, 0.0);

  Eigen::Quaterniond q(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);

  // A diverged localiser can publish a non-finite transform. Every point
  // derived from it would be NaN, poisoning the chunk the sensor sits in.
  // Drop the cloud rather than letting that reach the map.
  if (!origin.allFinite() || !q.coeffs().allFinite()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Transform %s -> %s is not finite; dropping cloud",
      msg->header.frame_id.c_str(), server_params_.frame_id.c_str());
    return;
  }

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(cloud->size());

  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    Eigen::Vector3d ps(p.x, p.y, p.z);
    Eigen::Vector3d pm = q * ps + origin;

    if (pm.z() < rolling_params_.map_params.occupancy_min_z ||
        pm.z() > rolling_params_.map_params.occupancy_max_z)
      continue;

    pts.push_back(pm);
  }

  if (!pts.empty()) {
    occupancy_map_->insertPointCloud(
      pts, origin, linvel,
      rolling_params_.map_params.sensor_max_range);
  }
}

} // namespace RollingBonxai


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RollingBonxai::RollingBonxaiServer)
