#ifndef BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_
#define BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_

// Exposes Pipeline::saveMap() as a std_srvs/Trigger service, so the accumulated
// registered cloud can be written to disk on demand:
//
//   ros2 service call /<node>/save_map std_srvs/srv/Trigger
//
// Trigger carries no request fields, so the destination comes from the YAML
// config (map_save.path, empty = ./bievr_map.pcd); the response reports the
// path actually written. The service callback runs on the node's executor,
// which is single-threaded in both wrapper executables, so it never overlaps
// with the pipeline callbacks that fill the map.

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
    srv_ = node->create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::string written;
          response->success = pipeline_->saveMap("", &written);
          response->message =
              response->success
                  ? "Saved " + std::to_string(pipeline_->accumulatedMapSize()) + " points to " +
                        written
                  : "Failed to save map; see the node log for the reason.";
        });
  }

 private:
  std::shared_ptr<const Pipeline> pipeline_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS2_SAVE_MAP_SERVICE_H_
