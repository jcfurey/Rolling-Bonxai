#pragma once

#include <memory>
#include <string>
#include <chrono>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// tf2 headers were renamed to .hpp in Jazzy; the legacy .h spellings are
// deprecation shims that are removed in later distros. Use .hpp so the same
// source builds on Jazzy and Lyrical.
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Geometry>

// Bonxai
#include "bonxai_map/occupancy_map.hpp"

// Custom services and messages
#include "bonxai_msgs/srv/get_occupied_voxels.hpp"
#include "bonxai_msgs/srv/get_free_voxels.hpp"
#include "bonxai_msgs/msg/occupancy_map_stats.hpp"

namespace Bonxai
{

struct BonxaiParams
{
  double resolution{0.0};
  std::string frame_id;
  std::string base_frame_id;
  std::string topic_in;

  double occupancy_min_z{0.0};
  double occupancy_max_z{0.0};
  double occupancy_threshold{0.50};

  double sensor_max_range{0.0};
  double sensor_hit{0.0};
  double sensor_miss{0.0};
  double sensor_min{0.0};
  double sensor_max{0.0};
  
  // Cleanup
  double cleanup_interval_sec{300.0};  // 5 minutes

  /// How long to block waiting for the sensor->map transform, in seconds
  double tf_timeout_sec{0.1};

  // Stats
  bool enable_stats{true};
  bool quick_stats{true};
  bool publish_occupied_voxels{true};
  double stats_publish_rate{1.0};  // Hz
  
};

class BonxaiServer : public rclcpp::Node
{
public:
  explicit BonxaiServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BonxaiServer() = default;

private:
  // Initialization methods
  void load_parameters();
  void init_tf();
  void init_publishers();
  void init_subscribers();
  void init_services();
  void init_timers();

  // Callbacks
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Create a timer driven by the node clock
   *
   * Deliberately not create_wall_timer(): wall timers run off the steady clock
   * and ignore /clock, so under use_sim_time they would fire at wall rate while
   * everything else in the graph follows simulated time.
   *
   * @param rate_hz Firing rate in Hz (must be > 0)
   * @param callback Callback to invoke
   */
  rclcpp::TimerBase::SharedPtr create_ros_timer(
    double rate_hz, std::function<void()> callback);


  // Service handlers
  void handle_get_occupied_voxels(
    const std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Request> request,
    std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Response> response);
    
  void handle_get_free_voxels(
    const std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Request> request,
    std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Response> response);
  
  // Timer callbacks
  void cleanup_timer_callback();
  void stats_timer_callback();

  // Helper: Fill VoxelGrid message from coordinate vector
  void fill_voxel_grid_msg(
    const std::vector<Bonxai::CoordT>& coords,
    bonxai_msgs::msg::VoxelGrid& msg);

  void fill_pcl_msg(
    const std::vector<Bonxai::CoordT>& coords,
    sensor_msgs::msg::PointCloud2& msg);
    
  // Helper: Fill OccupancyMapStats message
  void fill_stats_msg(bonxai_msgs::msg::OccupancyMapStats& msg);

  // Parameters
  BonxaiParams params_;

  // Flags
  bool updated_map_once_{false};

  // Occupancy map
  std::unique_ptr<Bonxai::OccupancyMap> occupancy_map_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Service servers
  rclcpp::Service<bonxai_msgs::srv::GetOccupiedVoxels>::SharedPtr occupied_voxels_service_;
  rclcpp::Service<bonxai_msgs::srv::GetFreeVoxels>::SharedPtr free_voxels_service_;
  
  // Publishers
  rclcpp::Publisher<bonxai_msgs::msg::OccupancyMapStats>::SharedPtr stats_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_voxel_publisher_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  // Statistics
  uint64_t point_clouds_processed_{0};
  uint64_t total_points_processed_{0};
};

}  // namespace Bonxai