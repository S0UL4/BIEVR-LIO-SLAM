#ifndef BIEVR_LIO_ROS2_LOCALIZATION_H_
#define BIEVR_LIO_ROS2_LOCALIZATION_H_

// Attaches bievr_localization to a running Pipeline and exposes its results:
//
//   TF <map> -> <odom>          the correction, broadcast only once localized
//   <prefix>/loc/odom           nav_msgs/Odometry, the pose in the map frame
//   <prefix>/loc/map            sensor_msgs/PointCloud2, the prior map
//   <prefix>/loc/status         std_msgs/String, NO_MAP/WAITING_FOR_POSE/...
//   /initialpose                subscribed, seeds or re-seeds localization
//
// The correction is published alongside the odometry and never fed back, so the
// odometry behaves identically whether this is running or not.
//
// TF is broadcast by a dedicated broadcaster rather than through
// bievr::Publisher: that one mirrors every Odometry it publishes onto TF, which
// here would fight the odometry for the same child frame.

#include <bievr_lio/pipeline.h>
#include <bievr_localization/config_loader.h>
#include <bievr_localization/localizer.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <utility>

#include "bievr_ros_common/conversions.h"

namespace bievr {

class Localization {
 public:
  Localization(rclcpp::Node::SharedPtr node, const std::shared_ptr<Pipeline>& pipeline,
               LocalizationConfig config, const std::string& topic_prefix)
      : node_(std::move(node)), config_(std::move(config)), pipeline_(pipeline) {
    localizer_ = std::make_unique<Localizer>(config_.localizer);
    odom_frame_ = pipeline->config().map_frame;
    body_frame_ = pipeline->config().body_frame;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/loc/odom", qos);
    map_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_prefix + "/loc/map", qos);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(topic_prefix + "/loc/status", qos);
    tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  // Loads the prior map and starts publishing. Returns false when the map cannot
  // be read, which is a configuration error rather than something to run past.
  bool start(std::string* message = nullptr) {
    if (!localizer_->start(message)) return false;

    // Republishing the whole prior map is heavy, so it goes off the default
    // group; the pose broadcast is cheap and stays serialized with the sensors.
    map_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          onInitialPose(*msg);
        });

    pose_timer_ = node_->create_wall_timer(periodFrom(config_.publish_frequency),
                                           [this] { publishPose(); });
    map_timer_ = node_->create_wall_timer(periodFrom(config_.map_publish_frequency),
                                          [this] { publishMap(); }, map_group_);

    // Registering last: nothing reaches the Localizer until it is fully built.
    pipeline_->addFrameObserver(
        [this](uint64_t stamp, const Transform& T_W_I, const Pointcloud& undistorted) {
          {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_ = T_W_I;
            latest_stamp_ = stamp;
            have_odom_ = true;
          }
          localizer_->addFrame(stamp, T_W_I, undistorted);
        });
    return true;
  }

  Localizer::Status status() const { return localizer_->status(); }
  Localizer::Stats stats() const { return localizer_->stats(); }
  bool canRelocalize() const { return localizer_->canRelocalize(); }

 private:
  static std::chrono::nanoseconds periodFrom(double hz) {
    const double seconds = hz > 0.0 ? 1.0 / hz : 1.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9));
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != config_.localizer.map_frame) {
      LOG(W, "Ignoring /initialpose in frame '" << msg.header.frame_id << "'; expected '"
                                                << config_.localizer.map_frame << "'.");
      return;
    }
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    localizer_->setInitialPose(Transform(Quaternion(q.w, q.x, q.y, q.z), Point(p.x, p.y, p.z)));
    LOG(I, "Accepted an initial pose at (" << p.x << ", " << p.y << ", " << p.z << ").");
  }

  void publishPose() {
    const Localizer::Status status = localizer_->status();
    if (status != last_status_) {
      LOG(I, "Localization status: " << Localizer::toString(status));
      last_status_ = status;
    }
    std_msgs::msg::String status_msg;
    status_msg.data = Localizer::toString(status);
    status_pub_->publish(status_msg);

    const Localizer::Correction correction = localizer_->correction();
    if (!correction.valid) return;  // no fix yet: better no TF than a fake identity

    Transform T_odom_body;
    uint64_t stamp = 0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_odom_) return;
      T_odom_body = latest_odom_;
      stamp = latest_stamp_;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp));
    tf_msg.header.frame_id = config_.localizer.map_frame;
    tf_msg.child_frame_id = odom_frame_;
    transformToMsg(correction.T_map_odom, tf_msg.transform);
    tf_->sendTransform(tf_msg);

    nav_msgs::msg::Odometry odom;
    odom.header = tf_msg.header;
    odom.child_frame_id = body_frame_;
    transformToMsg(Transform(Eigen::Isometry3d(correction.T_map_odom * T_odom_body)),
                   odom.pose.pose);
    odom_pub_->publish(odom);
  }

  void publishMap() {
    if (map_pub_->get_subscription_count() == 0) return;
    const Pointcloud map = localizer_->mapCloud();
    if (map.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pointCloudToMsg(map, msg);
    msg.header.frame_id = config_.localizer.map_frame;
    msg.header.stamp = node_->now();
    map_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  LocalizationConfig config_;
  std::shared_ptr<Pipeline> pipeline_;
  std::unique_ptr<Localizer> localizer_;

  std::string odom_frame_;  // the odometry's parent frame, child of the correction
  std::string body_frame_;
  Localizer::Status last_status_ = Localizer::Status::NoMap;

  mutable std::mutex odom_mutex_;
  Transform latest_odom_;
  uint64_t latest_stamp_ = 0;
  bool have_odom_ = false;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::CallbackGroup::SharedPtr map_group_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS2_LOCALIZATION_H_
