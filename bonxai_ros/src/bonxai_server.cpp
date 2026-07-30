#include "bonxai_ros/bonxai_server.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <chrono>
#include <utility>

namespace Bonxai
{

BonxaiServer::BonxaiServer(const rclcpp::NodeOptions& options)
  : Node("bonxai_server_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing BonxaiServer...");
  
  // Initialize in order
  load_parameters();
  init_tf();
  
  // Create occupancy map
  Bonxai::Occupancy::OccupancyOptions occ_options;
  occ_options.prob_hit_log = Bonxai::Occupancy::logods(params_.sensor_hit);
  occ_options.prob_miss_log = Bonxai::Occupancy::logods(params_.sensor_miss);
  occ_options.clamp_min_log = Bonxai::Occupancy::logods(params_.sensor_min);
  occ_options.clamp_max_log = Bonxai::Occupancy::logods(params_.sensor_max);
  occ_options.occupancy_threshold_log = Bonxai::Occupancy::logods(params_.occupancy_threshold);
  
  occupancy_map_ = std::make_unique<Bonxai::OccupancyMap>(params_.resolution, occ_options);
  
  RCLCPP_INFO(get_logger(), "OccupancyMap created with resolution: %.3f", params_.resolution);
  
  // Initialize components
  init_publishers();
  init_services();
  init_subscribers();
  init_timers();

  // use_sim_time is pre-declared by rclcpp. Report it, because almost every
  // "the map never updates" report against a simulated robot comes down to this
  // being false while /clock is the only time source in the graph.
  const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  RCLCPP_INFO(
    get_logger(),
    "BonxaiServer initialization complete! (use_sim_time=%s, clock=%s)",
    use_sim_time ? "true" : "false",
    use_sim_time ? "/clock" : "system");
}

rclcpp::TimerBase::SharedPtr BonxaiServer::create_ros_timer(
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

  return rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration(period), std::move(callback));
}

void BonxaiServer::load_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading parameters...");
  
  // --- Core map parameters ---

  params_.frame_id =
    this->declare_parameter<std::string>("frame_id", "map");
    
  params_.base_frame_id =
    this->declare_parameter<std::string>("base_frame_id", "base_footprint");
    
  params_.topic_in =
    this->declare_parameter<std::string>("topic_in", "/points");

  params_.resolution =
    this->declare_parameter<double>("occupancy.resolution", 0.05);
  
  // --- Occupancy z-bounds ---
  params_.occupancy_min_z =
    this->declare_parameter<double>("occupancy.occupancy_min_z", -1.0);
    
  params_.occupancy_max_z =
    this->declare_parameter<double>("occupancy.occupancy_max_z", 2.0);

  // --- Occupancy Threshold ---
  params_.occupancy_threshold =
    this->declare_parameter<double>("occupancy.occupancy_threshold", 0.50);
  
  // --- Sensor model parameters ---
  params_.sensor_max_range =
    this->declare_parameter<double>("sensor_model.max_range", 30.0);
    
  params_.sensor_hit =
    this->declare_parameter<double>("sensor_model.hit", 0.7);
    
  params_.sensor_miss =
    this->declare_parameter<double>("sensor_model.miss", 0.4);
    
  params_.sensor_min =
    this->declare_parameter<double>("sensor_model.min", 0.12);
    
  params_.sensor_max =
    this->declare_parameter<double>("sensor_model.max", 0.97);
  
  // --- Server ---
  params_.cleanup_interval_sec =
    this->declare_parameter<double>("server.cleanup_interval_sec", 300.0);

  params_.tf_timeout_sec =
    this->declare_parameter<double>("server.tf_timeout_sec", 0.1);

  // --- Statostics ---
  params_.enable_stats =
    this->declare_parameter<bool>("stats.enable_stats", true);
    
  params_.quick_stats =
    this->declare_parameter<bool>("stats.quick_stats", true);

  params_.publish_occupied_voxels = this->declare_parameter<bool>("stats.publish_occupied_voxels",true);
    
  params_.stats_publish_rate =
    this->declare_parameter<double>("stats.publish_rate", 1.0);

  RCLCPP_INFO(get_logger(),
    "Bonxai params loaded:"
    "\n  resolution=%.3f"
    "\n  frame_id=%s"
    "\n  base_frame_id=%s"
    "\n  topic_in=%s"
    "\n  occupancy_z=[%.2f, %.2f]"
    "\n  occupancy_thresh = %.2f"
    "\n  sensor(hit=%.2f miss=%.2f min=%.2f max=%.2f range=%.2f)"
    "\n  cleanup_interval=%.1fs"
    "\n  stats(enabled=%s quick=%s voxels=%s rate=%.1fHz)",
    params_.resolution,
    params_.frame_id.c_str(),
    params_.base_frame_id.c_str(),
    params_.topic_in.c_str(),
    params_.occupancy_min_z,
    params_.occupancy_max_z,
    params_.occupancy_threshold,
    params_.sensor_hit,
    params_.sensor_miss,
    params_.sensor_min,
    params_.sensor_max,
    params_.sensor_max_range,
    params_.cleanup_interval_sec,
    params_.enable_stats ? "true" : "false",
    params_.quick_stats ? "true" : "false",
    params_.publish_occupied_voxels ? "true" : "false",
    params_.stats_publish_rate);
}

void BonxaiServer::init_tf()
{
  RCLCPP_INFO(get_logger(), "Initializing TF...");
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

  // Give the buffer a ROS-time timer factory. Without it, any timeout-based
  // wait inside tf2 falls back to a steady-clock wait, which does not line up
  // with the sim-time stamps on incoming transforms.
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "TF initialized");
}

void BonxaiServer::init_publishers()
{
  RCLCPP_INFO(get_logger(), "Initializing publishers...");
  
  // Create stats publisher if enabled
  if (params_.enable_stats) {
    stats_publisher_ = this->create_publisher<bonxai_msgs::msg::OccupancyMapStats>(
      "/bonxai/occupancy_stats",
      rclcpp::QoS(10));
    
    RCLCPP_INFO(get_logger(), "Stats publisher created: /bonxai/occupancy_stats");

    // stats.publish_occupied_voxels was declared and logged but never checked,
    // so the occupied-voxel cloud was published unconditionally — and building
    // it walks every active cell on each stats tick, which is exactly the cost
    // this switch exists to avoid.
    if (params_.publish_occupied_voxels) {
      occupied_voxel_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/bonxai/occupied_voxels", rclcpp::QoS(10));

      RCLCPP_INFO(get_logger(), "Occupied voxel publisher created: /bonxai/occupied_voxels");
    }
  }
}

void BonxaiServer::init_subscribers()
{
  RCLCPP_INFO(get_logger(), "Initializing subscribers...");
  
  // Subscribe to point cloud
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.topic_in,
    rclcpp::SensorDataQoS(),
    std::bind(&BonxaiServer::pointcloud_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(get_logger(), "Subscribed to: %s", params_.topic_in.c_str());
}

void BonxaiServer::init_services()
{
  RCLCPP_INFO(get_logger(), "Initializing services...");
  
  // Create service servers
  occupied_voxels_service_ = this->create_service<bonxai_msgs::srv::GetOccupiedVoxels>(
    "/bonxai/get_occupied_voxels",
    std::bind(&BonxaiServer::handle_get_occupied_voxels, this,
              std::placeholders::_1, std::placeholders::_2));
              
  free_voxels_service_ = this->create_service<bonxai_msgs::srv::GetFreeVoxels>(
    "/bonxai/get_free_voxels",
    std::bind(&BonxaiServer::handle_get_free_voxels, this,
              std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(get_logger(), "Services created:");
  RCLCPP_INFO(get_logger(), "  - /bonxai/get_occupied_voxels");
  RCLCPP_INFO(get_logger(), "  - /bonxai/get_free_voxels");
}

void BonxaiServer::init_timers()
{
  RCLCPP_INFO(get_logger(), "Initializing timers...");
  
  // Start cleanup timer
  if (params_.cleanup_interval_sec > 0.0) {
    cleanup_timer_ = create_ros_timer(
      1.0 / params_.cleanup_interval_sec,
      std::bind(&BonxaiServer::cleanup_timer_callback, this));

    RCLCPP_INFO(get_logger(), "Cleanup timer started: every %.1fs", params_.cleanup_interval_sec);
  } else {
    RCLCPP_INFO(get_logger(), "Cleanup timer disabled (server.cleanup_interval_sec <= 0)");
  }

  // Start stats timer if enabled
  if (params_.enable_stats && stats_publisher_) {
    stats_timer_ = create_ros_timer(
      params_.stats_publish_rate,
      std::bind(&BonxaiServer::stats_timer_callback, this));


    RCLCPP_INFO(get_logger(), 
      "Stats publishing enabled: rate=%.1f Hz, quick=%s",
      params_.stats_publish_rate,
      params_.quick_stats ? "true" : "false");
  }
}

void BonxaiServer::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!occupancy_map_) {
    RCLCPP_WARN(get_logger(), "Occupancy map not initialized yet");
    return;
  }

  // An all-zero stamp means the publisher never filled the header. Asking tf2
  // for transform-at-time-zero is interpreted as "the latest available", which
  // is the only sensible reading; looking up an actual epoch-zero timestamp
  // would fail on every cloud.
  const rclcpp::Time stamp(msg->header.stamp);
  const tf2::TimePoint lookup_time =
    (stamp.nanoseconds() == 0)
      ? tf2::TimePointZero
      : tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds()));

  // Look up transform from sensor frame to map frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      params_.frame_id, //target frame: the global map frame
      msg->header.frame_id, //frame where the camera is publishing at
      lookup_time,
      tf2::durationFromSec(params_.tf_timeout_sec));
  } catch (const tf2::TransformException& ex) {
    // Throttled: on a mis-configured frame this fires at full sensor rate.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF lookup %s -> %s failed: %s",
      msg->header.frame_id.c_str(), params_.frame_id.c_str(), ex.what());
    return;
  }


  // Convert ROS message to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  
  if (cloud->empty()) {
    RCLCPP_DEBUG(get_logger(), "Received empty point cloud");
    return;
  }
  
  // Extract translation (sensor origin in map frame)
  Eigen::Vector3d sensor_origin(
    transform_stamped.transform.translation.x,
    transform_stamped.transform.translation.y,
    transform_stamped.transform.translation.z
  );
  
  // Extract rotation
  Eigen::Quaterniond rotation(
    transform_stamped.transform.rotation.w,
    transform_stamped.transform.rotation.x,
    transform_stamped.transform.rotation.y,
    transform_stamped.transform.rotation.z
  );

  // tf2 screens NaN translations at ingestion (TF_NAN_INPUT) but does not
  // screen infinities, so an infinite transform from a diverging state
  // estimator reaches us intact. Every point derived from it would be
  // non-finite, poisoning the map. Drop the cloud instead.
  if (!sensor_origin.allFinite() || !rotation.coeffs().allFinite()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Transform %s -> %s is not finite; dropping cloud",
      msg->header.frame_id.c_str(), params_.frame_id.c_str());
    return;
  }


  // Transform points to map frame and filter by Z
  std::vector<Eigen::Vector3d> map_points;
  map_points.reserve(cloud->size());
  
  for (const auto& point : cloud->points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Transform to map frame
    Eigen::Vector3d point_sensor(point.x, point.y, point.z);
    Eigen::Vector3d point_map = rotation * point_sensor + sensor_origin;
    
    // Filter by Z bounds
    if (point_map.z() < params_.occupancy_min_z || point_map.z() > params_.occupancy_max_z) {
      continue;
    }
    
    map_points.push_back(point_map);
  }
  
  if (map_points.empty()) {
    RCLCPP_DEBUG(get_logger(), "No valid points after filtering");
    return;
  }
  
  // Insert into occupancy map
  occupancy_map_->insertPointCloud(map_points, sensor_origin, params_.sensor_max_range);
  if (!updated_map_once_){
    updated_map_once_ = true;
  }
  
  // Update statistics
  point_clouds_processed_++;
  total_points_processed_ += map_points.size();
  
  RCLCPP_DEBUG(get_logger(), 
    "Processed cloud: %zu points, %zu after filtering, origin: [%.2f, %.2f, %.2f]",
    cloud->size(), map_points.size(),
    sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
}

void BonxaiServer::handle_get_occupied_voxels(
  const std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Request> request,
  std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Response> response)
{
  (void)request;  // Unused
  
  if (!occupancy_map_ || !updated_map_once_) {
    RCLCPP_ERROR(get_logger(), "Occupancy map not initialized");
    response->success = false;
    return;
  }
  
  auto start = std::chrono::steady_clock::now();
  
  // Get occupied voxels
  std::vector<Bonxai::CoordT> coords;
  occupancy_map_->getOccupiedVoxels(coords);
  
  // Fill response
  fill_voxel_grid_msg(coords, response->voxel_grid);
  
  // Get statistics
  auto stats = occupancy_map_->getQuickStats();
  response->occupied_count = stats.occupied_cells;
  response->total_active_cells = stats.total_active_cells;
  response->occupancy_ratio = stats.occupancy_ratio;
  response->memory_mb = static_cast<float>(stats.total_memory_bytes) / 1048576.0f;
  
  auto end = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  
  RCLCPP_INFO(get_logger(),
    "GetOccupiedVoxels: %zu voxels, %.1f MB, took %ld ms",
    coords.size(), response->memory_mb, duration_ms);
}

void BonxaiServer::handle_get_free_voxels(
  const std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Request> request,
  std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Response> response)
{
  (void)request;  // Unused
  
  if (!occupancy_map_ || !updated_map_once_) {
    RCLCPP_ERROR(get_logger(), "Occupancy map not initialized");
    response->success = false;
    return;
  }
  
  auto start = std::chrono::steady_clock::now();
  
  // Get free voxels
  std::vector<Bonxai::CoordT> coords;
  occupancy_map_->getFreeVoxels(coords);
  
  // Fill response
  fill_voxel_grid_msg(coords, response->voxel_grid);
  
  // Get statistics
  auto stats = occupancy_map_->getQuickStats();
  response->free_count = stats.free_cells;
  response->total_active_cells = stats.total_active_cells;
  response->occupancy_ratio = stats.occupancy_ratio;
  response->memory_mb = static_cast<float>(stats.total_memory_bytes) / 1048576.0f;
  
  auto end = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  
  RCLCPP_INFO(get_logger(),
    "GetFreeVoxels: %zu voxels, %.1f MB, took %ld ms",
    coords.size(), response->memory_mb, duration_ms);
}

void BonxaiServer::fill_voxel_grid_msg(
  const std::vector<Bonxai::CoordT>& coords,
  bonxai_msgs::msg::VoxelGrid& msg)
{
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.frame_id;
  msg.resolution = params_.resolution;
  msg.num_voxels = static_cast<uint32_t>(coords.size());
  
  if (coords.empty()) {
    msg.min_x = msg.min_y = msg.min_z = 0;
    msg.max_x = msg.max_y = msg.max_z = 0;
    return;
  }
  
  // Reserve space
  msg.x.reserve(coords.size());
  msg.y.reserve(coords.size());
  msg.z.reserve(coords.size());
  
  // Initialize bounding box
  int32_t min_x = coords[0].x, max_x = coords[0].x;
  int32_t min_y = coords[0].y, max_y = coords[0].y;
  int32_t min_z = coords[0].z, max_z = coords[0].z;
  
  // Fill parallel arrays and compute bounding box
  for (const auto& coord : coords) {
    msg.x.push_back(coord.x);
    msg.y.push_back(coord.y);
    msg.z.push_back(coord.z);
    
    min_x = std::min(min_x, coord.x);
    max_x = std::max(max_x, coord.x);
    min_y = std::min(min_y, coord.y);
    max_y = std::max(max_y, coord.y);
    min_z = std::min(min_z, coord.z);
    max_z = std::max(max_z, coord.z);
  }
  
  msg.min_x = min_x;
  msg.min_y = min_y;
  msg.min_z = min_z;
  msg.max_x = max_x;
  msg.max_y = max_y;
  msg.max_z = max_z;
}

void BonxaiServer::fill_stats_msg(bonxai_msgs::msg::OccupancyMapStats& msg)
{
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.frame_id;
  
  // Get stats from occupancy map
  Bonxai::Occupancy::OccupancyMapStats stats;
  
  if (params_.quick_stats) {
    stats = occupancy_map_->getQuickStats();
    msg.expensive_stats_computed = false;
  } else {
    stats = occupancy_map_->getStats(true);
    msg.expensive_stats_computed = true;
  }
  
  // Memory usage
  msg.total_memory_mib =  static_cast<double>(stats.total_memory_bytes) / 1048576.0f;
  msg.grid_memory_mib =   static_cast<double>(stats.grid_memory_bytes) / 1048576.0f;
  msg.buffer_memory_mib = static_cast<double>(stats.buffer_memory_bytes) / 1048576.0f;
  
  // Cell counts
  msg.total_active_cells = stats.total_active_cells;
  msg.occupied_cells = stats.occupied_cells;
  msg.free_cells = stats.free_cells;
  msg.unknown_cells = stats.unknown_cells;
  
  // Update statistics
  msg.total_insertions = stats.total_insertions;
  msg.hit_points_added = stats.hit_points_added;
  msg.miss_points_added = stats.miss_points_added;
  msg.current_update_cycle = stats.current_update_cycle;
  
  // Spatial extent
  msg.has_data = stats.has_data;
  if (stats.has_data) {
    msg.min_x = stats.min_coord.x;
    msg.min_y = stats.min_coord.y;
    msg.min_z = stats.min_coord.z;
    msg.max_x = stats.max_coord.x;
    msg.max_y = stats.max_coord.y;
    msg.max_z = stats.max_coord.z;
  } else {
    msg.min_x = msg.min_y = msg.min_z = 0;
    msg.max_x = msg.max_y = msg.max_z = 0;
  }
  
  // Probability distribution (only if expensive stats computed)
  if (msg.expensive_stats_computed) {
    msg.min_probability_log = stats.min_probability_log;
    msg.max_probability_log = stats.max_probability_log;
    msg.mean_probability_log = stats.mean_probability_log;
  } else {
    msg.min_probability_log = 0;
    msg.max_probability_log = 0;
    msg.mean_probability_log = 0;
  }
  
  // Derived metrics
  msg.occupancy_ratio = stats.occupancy_ratio;
  msg.exploration_ratio = stats.exploration_ratio;
  
  // Performance metrics (from this node)
  msg.point_clouds_processed = point_clouds_processed_;
  msg.total_points_processed = total_points_processed_;
}

void BonxaiServer::cleanup_timer_callback()
{
  if (!occupancy_map_) {
    return;
  }
  
  auto stats_before = occupancy_map_->getQuickStats();
  
  occupancy_map_->releaseUnusedMemory();
  
  auto stats_after = occupancy_map_->getQuickStats();
  
  double mb_before = static_cast<double>(stats_before.total_memory_bytes) / 1048576.0;
  double mb_after = static_cast<double>(stats_after.total_memory_bytes) / 1048576.0;
  double mb_freed = mb_before - mb_after;
  
  RCLCPP_INFO(get_logger(),
    "Cleanup: memory %.1f MB → %.1f MB (freed %.1f MB), "
    "clouds processed: %lu, points: %lu",
    mb_before, mb_after, mb_freed,
    point_clouds_processed_, total_points_processed_);
}

void BonxaiServer::fill_pcl_msg(
    const std::vector<Bonxai::CoordT>& coords,
   sensor_msgs::msg::PointCloud2& pcl_msg)
{
  // Set up the point cloud message
  pcl_msg.header.stamp = this->now();
  pcl_msg.header.frame_id = params_.frame_id;

  // Use PointCloud2Modifier to set up fields and reserve space
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");  // Add RGB!
  modifier.resize(coords.size());

  // Create iterator for filling data
sensor_msgs::PointCloud2Iterator<float> iter_x(pcl_msg, "x");
sensor_msgs::PointCloud2Iterator<float> iter_y(pcl_msg, "y");
sensor_msgs::PointCloud2Iterator<float> iter_z(pcl_msg, "z");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pcl_msg, "r");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pcl_msg, "g");
sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pcl_msg, "b");

  // Get grid for coordinate conversion

  const auto &underlying_grid = occupancy_map_->getGrid();
  // Fill in the data
  for (const auto& coord : coords) {
      auto pos = underlying_grid.coordToPos(coord);
      
      *iter_x = static_cast<float>(pos.x);
      *iter_y = static_cast<float>(pos.y);
      *iter_z = static_cast<float>(pos.z);
      
      // Set color (e.g., red for occupied)
      *iter_r = 255;
      *iter_g = 0;
      *iter_b = 0;
      
      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
  }
}

void BonxaiServer::stats_timer_callback()
{
  if (!occupancy_map_ || !stats_publisher_) {
    return;
  }
  
  std::vector<Bonxai::CoordT> coords;
  if (occupied_voxel_publisher_) {
    sensor_msgs::msg::PointCloud2 pcl_msg;
    occupancy_map_->getOccupiedVoxels(coords);
    fill_pcl_msg(coords, pcl_msg);
    occupied_voxel_publisher_->publish(pcl_msg);
  }

  bonxai_msgs::msg::OccupancyMapStats msg;
  fill_stats_msg(msg);
  
  stats_publisher_->publish(msg);

  // Debug: this runs at stats_publish_rate (3 Hz by default), and at info level
  // it buries everything else in the log.
  RCLCPP_DEBUG(get_logger(),
    "Stats published: %zu occupied voxels, %lu active cells, %.1f MiB, occupancy=%.2f%%",
    coords.size(),
    static_cast<unsigned long>(msg.total_active_cells),
    static_cast<double>(msg.total_memory_mib),
    static_cast<double>(msg.occupancy_ratio) * 100.0);
}

}  // namespace Bonxai

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Bonxai::BonxaiServer)