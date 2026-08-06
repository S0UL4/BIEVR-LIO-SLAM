#ifndef BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_
#define BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_

// Exposes Pipeline::saveMap() as a std_srvs/Trigger service, so the accumulated
// registered cloud can be written to disk on demand:
//
//   ros2 service call /<node>/save_map std_srvs/srv/Trigger
//
// Trigger carries no request fields, so the destination comes from the YAML
// config (map_save.path, empty = ./bievr_map.pcd); the response reports the
// path actually written. Writing a long session's map takes seconds, so the
// callback gets its own group rather than the node's default one: under a
// MultiThreadedExecutor it then runs alongside the sensor callbacks instead of
// stalling the odometry. MapAccumulator::save copies the map under its lock and
// writes outside it, so overlapping with a live feed is safe.

#include <bievr_lio/pipeline.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <utility>

namespace bievr {

class SaveMapService {
 public:
  SaveMapService(rclcpp::Node::SharedPtr node, std::shared_ptr<const Pipeline> pipeline)
      : pipeline_(std::move(pipeline)) {
    group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = node->create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::string written;
          size_t num_points = 0;
          response->success = pipeline_->saveMap("", &written, &num_points);
          response->message =
              response->success
                  ? "Saved " + std::to_string(num_points) + " points to " + written
                  : "Failed to save map; see the node log for the reason.";
        },
        rclcpp::ServicesQoS(), group_);
  }

 private:
  std::shared_ptr<const Pipeline> pipeline_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_
