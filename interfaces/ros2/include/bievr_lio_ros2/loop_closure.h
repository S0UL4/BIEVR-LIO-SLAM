#ifndef BIEVR_LIO_ROS2_LOOP_CLOSURE_H_
#define BIEVR_LIO_ROS2_LOOP_CLOSURE_H_

// Attaches bievr_pgo to a running Pipeline and exposes its results on ROS:
//
//   <prefix>/pgo/path   nav_msgs/Path         loop-corrected keyframe trajectory
//   <prefix>/pgo/odom   nav_msgs/Odometry     latest corrected keyframe pose
//   <prefix>/pgo/map    sensor_msgs/PointCloud2
//   ~/save_map_bundle   std_srvs/Trigger      writes the map bundle to disk
//
// The corrected poses are published alongside the odometry, never fed back into
// it, so the odometry behaves identically whether this is running or not.

#include <bievr_lio/pipeline.h>
#include <bievr_pgo/config_loader.h>
#include <bievr_pgo/loop_closer.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <utility>

#include "bievr_ros_common/conversions.h"

namespace bievr {

class LoopClosure {
 public:
  LoopClosure(rclcpp::Node::SharedPtr node, const std::shared_ptr<Pipeline>& pipeline,
              LoopClosureConfig config, const std::string& topic_prefix)
      : node_(std::move(node)), config_(std::move(config)) {
    closer_ = std::make_unique<LoopCloser>(config_.closer);
    bundle_path_ = config_.bundle_path;
    child_frame_ = pipeline->config().body_frame;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/pgo/path", qos);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/pgo/odom", qos);
    map_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_prefix + "/pgo/map", qos);

    // Saving and map publishing both rebuild the whole map, which takes seconds
    // on a long session. Off the default group so they cannot stall the sensor
    // callbacks (the odometry runs inline in them); mutually exclusive so the
    // two never rebuild at once. Needs a MultiThreadedExecutor to take effect.
    heavy_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    save_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/save_map_bundle",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::string message;
          response->success = closer_->saveMapBundle(bundle_path_, &message);
          response->message = message;
        },
        rclcpp::ServicesQoS(), heavy_group_);

    path_timer_ = node_->create_wall_timer(periodFrom(config_.path_publish_frequency),
                                           [this] { publishPath(); });
    map_timer_ = node_->create_wall_timer(periodFrom(config_.map_publish_frequency),
                                          [this] { publishMap(); }, heavy_group_);

    // Registering last: nothing reaches the LoopCloser until it is fully built.
    pipeline->addFrameObserver(
        [this](uint64_t stamp, const Transform& T_W_I, const Pointcloud& undistorted) {
          closer_->addFrame(stamp, T_W_I, undistorted);
        });
  }

  LoopCloser::Stats stats() const { return closer_->stats(); }
  bool idle() const { return closer_->idle(); }
  bool saveBundle(std::string* message) const {
    return closer_->saveMapBundle(bundle_path_, message);
  }
  const std::string& bundlePath() const { return bundle_path_; }

 private:
  static std::chrono::nanoseconds periodFrom(double hz) {
    const double seconds = hz > 0.0 ? 1.0 / hz : 1.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9));
  }

  void publishPath() {
    const auto keyframes = closer_->keyframes();
    if (keyframes.empty()) return;

    nav_msgs::msg::Path path;
    path.header.frame_id = config_.closer.map_frame;
    path.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframes.back().stamp));
    path.poses.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = rclcpp::Time(static_cast<int64_t>(kf.stamp));
      transformToMsg(kf.pose, pose.pose);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);

    nav_msgs::msg::Odometry odom;
    odom.header = path.header;
    odom.child_frame_id = child_frame_;
    transformToMsg(keyframes.back().pose, odom.pose.pose);
    odom_pub_->publish(odom);
  }

  void publishMap() {
    if (map_pub_->get_subscription_count() == 0) return;
    const Pointcloud map = closer_->buildMap(config_.closer.map_save_resolution);
    if (map.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pointCloudToMsg(map, msg);
    msg.header.frame_id = config_.closer.map_frame;
    msg.header.stamp = node_->now();
    map_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  LoopClosureConfig config_;
  std::unique_ptr<LoopCloser> closer_;
  std::string bundle_path_ = "bievr_map_bundle";
  std::string child_frame_;  // from the pipeline's body_frame

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::CallbackGroup::SharedPtr heavy_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS2_LOOP_CLOSURE_H_
