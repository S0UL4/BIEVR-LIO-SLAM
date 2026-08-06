#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros2/publisher.h"
#include "bievr_lio_ros2/save_map_service.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_PGO
#include "bievr_lio_ros2/loop_closure.h"
#endif
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace {
template <typename T>
T deserialize(const rclcpp::SerializedMessage& serialized) {
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &msg);
  return msg;
}
}  // namespace

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bievr_lio_bag_node");

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
  std::unique_ptr<bievr::LoopClosure> loop_closure;
  bievr::LoopClosureConfig loop_closure_config;
  if (!bievr::loadLoopClosureConfig(config.yaml_paths, loop_closure_config)) {
    LOG(E, "Failed to load loop closure config.");
    return -1;
  }
  if (loop_closure_config.enable) {
    loop_closure =
        std::make_unique<bievr::LoopClosure>(node, pipeline, loop_closure_config, "bievr_lio");
    LOG(I, "Loop closure enabled.");
  }
#endif

  rosbag2_cpp::Reader reader;
  reader.open(config.topic_config.bag_path);

  // Map each topic to its message type (needed to pick the right deserializer).
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  const std::string& imu_topic = config.topic_config.imu_topic;

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    if (topic != pc_topic && topic != imu_topic) {
      continue;
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    const std::string& type = topic_types[topic];

    if (type == "sensor_msgs/msg/PointCloud2") {
      auto msg = deserialize<sensor_msgs::msg::PointCloud2>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(msg, pointcloud);
      synchronizer->addPointcloud(pointcloud);
    }
#ifdef BIEVR_WITH_LIVOX
    else if (type == "livox_ros_driver2/msg/CustomMsg") {
      auto msg = deserialize<livox_ros_driver2::msg::CustomMsg>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(msg, pointcloud);
      synchronizer->addPointcloud(pointcloud);
    }
#endif
    else if (type == "sensor_msgs/msg/Imu") {
      auto msg = deserialize<sensor_msgs::msg::Imu>(serialized);
      bievr::ImuMeasurement imu;
      bievr::msgToImuMeasurement(msg, imu);
      synchronizer->addImu(imu);
    }

    // Bag playback never spins, so serve any pending save_map call here.
    rclcpp::spin_some(node);
  }

  LOG(I, "Done with bag.");
  reader.close();
  LOG(I, "Bag closed");

  // The node exits as soon as the bag is exhausted, which leaves no chance to
  // call the service afterwards. Save on the way out when a destination was
  // configured -- the same "empty path means don't write" rule the trajectory
  // log uses.
  if (!config.pipeline_config.map_save_path.empty()) {
    pipeline->saveMap("");
  }

#ifdef BIEVR_WITH_PGO
  // The loop closure workers lag the feed, so let the backlog drain before
  // writing the bundle, then save on the way out for the same reason.
  if (loop_closure && !loop_closure->bundlePath().empty()) {
    LOG(I, "Waiting for loop closure to finish...");
    // Detection only ticks at loop_detect_frequency, so a momentarily empty
    // pipeline is not the end of the work. Require it to stay idle for longer
    // than one detection period before believing it.
    const int stable_polls =
        std::max(2, static_cast<int>(15.0 / std::max(loop_closure_config.closer
                                                         .loop_detect_frequency, 0.1)));
    int idle_polls = 0;
    for (int i = 0; i < 900 && idle_polls < stable_polls; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      idle_polls = loop_closure->idle() ? idle_polls + 1 : 0;
    }
    const auto stats = loop_closure->stats();
    LOG(I, "Loop closure: " << stats.num_keyframes << " keyframes, " << stats.num_loops
                            << " loops accepted, " << stats.num_rejected << " rejected, "
                            << stats.dropped_frames << " frames dropped.");
    std::string message;
    if (loop_closure->saveBundle(&message)) {
      LOG(I, message);
    } else {
      LOG(E, "Map bundle not saved: " << message);
    }
  }
#endif

  rclcpp::shutdown();
  return 0;
}
