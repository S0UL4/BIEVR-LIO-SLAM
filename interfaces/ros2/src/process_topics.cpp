#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros2/publisher.h"
#include "bievr_lio_ros2/save_map_service.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#ifdef BIEVR_WITH_PGO
#include "bievr_lio_ros2/loop_closure.h"
#endif
#ifdef BIEVR_WITH_LOCALIZATION
#include "bievr_lio_ros2/localization.h"
#endif

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bievr_lio_topic_node");

  bievr::Config config;
  // rclcpp::init does not strip ROS arguments from argv; remove_ros_arguments
  // returns just the application arguments (our config-file flags).
  if (!bievr::loadConfigFromArgs(rclcpp::remove_ros_arguments(argc, argv), config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  auto pipeline = std::make_shared<bievr::Pipeline>(config.pipeline_config);
  auto synchronizer = std::make_shared<bievr::Synchronizer>(pipeline);
  auto lio_pub = std::make_shared<bievr::Publisher>(node, pipeline, "bievr_lio");
  bievr::SaveMapService save_map_srv(node, pipeline);

#ifdef BIEVR_WITH_PGO
  // Loop closure is a downstream observer: off unless `loop_closure.enable` is
  // set, and it never writes back into the odometry.
  std::unique_ptr<bievr::LoopClosure> loop_closure;
  bievr::LoopClosureConfig loop_closure_config;
  if (!bievr::loadLoopClosureConfig(config.yaml_paths, loop_closure_config)) {
    LOG(E, "Failed to load loop closure config.");
    return -1;
  }
  if (loop_closure_config.enable) {
    loop_closure = std::make_unique<bievr::LoopClosure>(node, pipeline, loop_closure_config,
                                                        "bievr_lio");
    LOG(I, "Loop closure enabled.");
  }
#endif

#ifdef BIEVR_WITH_LOCALIZATION
  // Localization is the other downstream observer: it tracks a prior map and
  // publishes map -> odom, never correcting the odometry itself.
  std::unique_ptr<bievr::Localization> localization;
  bievr::LocalizationConfig localization_config;
  if (!bievr::loadLocalizationConfig(config.yaml_paths, localization_config)) {
    LOG(E, "Failed to load localization config.");
    return -1;
  }
  if (localization_config.enable) {
    localization = std::make_unique<bievr::Localization>(node, pipeline, localization_config,
                                                         "bievr_lio");
    std::string message;
    if (!localization->start(&message)) {
      LOG(E, "Localization enabled but its prior map could not be loaded: " << message);
      return -1;
    }
    LOG(I, "Localization enabled. " << message);
  }
#endif

  // ROS2 has no ShapeShifter: discover the pointcloud topic's type from the
  // graph, then use a generic (serialized) subscription to handle whichever of
  // the supported message types is being published on that single topic.
  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  std::string pc_type;
  LOG(I, "Waiting to discover type of pointcloud topic '" << pc_topic << "'...");
  while (rclcpp::ok() && pc_type.empty()) {
    const auto names_types = node->get_topic_names_and_types();
    const auto it = names_types.find(pc_topic);
    if (it != names_types.end() && !it->second.empty()) {
      pc_type = it->second.front();
    } else {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(100ms);
    }
  }
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 0;
  }
  LOG(I, "Pointcloud topic type: " << pc_type);

  // QoS must be compatible with the publisher. SensorDataQoS (best-effort)
  // matches most LiDAR drivers (e.g. Ouster); some drivers (e.g. Livox) publish
  // reliable — adjust here if you see no messages arriving.
  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

  auto pc_sub = node->create_generic_subscription(
      pc_topic, pc_type, sensor_qos,
      [&, pc_type](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        if (pc_type == "sensor_msgs/msg/PointCloud2") {
          sensor_msgs::msg::PointCloud2 msg;
          rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(
              serialized.get(), &msg);
          bievr::StampedIntensityPointcloud pointcloud;
          if (bievr::msgToPointcloud(msg, pointcloud)) {
            synchronizer->addPointcloud(pointcloud);
          }
        }
#ifdef BIEVR_WITH_LIVOX
        else if (pc_type == "livox_ros_driver2/msg/CustomMsg") {
          livox_ros_driver2::msg::CustomMsg msg;
          rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg>().deserialize_message(
              serialized.get(), &msg);
          bievr::StampedIntensityPointcloud pointcloud;
          if (bievr::msgToPointcloud(msg, pointcloud)) {
            synchronizer->addPointcloud(pointcloud);
          }
        }
#endif
        else {
          LOG_FIRST(W, 1, "Received unsupported pointcloud message type: " << pc_type);
        }
      });

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      config.topic_config.imu_topic, sensor_qos, [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        bievr::ImuMeasurement imu;
        if (!bievr::msgToImuMeasurement(*msg, imu)) {
          return;
        }
        synchronizer->addImu(imu);
      });

  // Multi-threaded so the loop closure's map save and map publish can run off
  // the sensor callbacks. Everything else stays in the node's default
  // (mutually exclusive) group, so it is still serialized exactly as before.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
