#ifndef BIEVR_LIO_ROS2_PUBLISHER_H_
#define BIEVR_LIO_ROS2_PUBLISHER_H_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <typeindex>
#include <typeinfo>

// publisher_base.h holds the shared publish logic and transitively includes
// bievr_ros_common/conversions.h (which pulls in the ROS message headers).
#include "bievr_ros_common/publisher_base.h"

namespace bievr {

// Type-erased wrapper around a typed rclcpp::Publisher. ROS2 publishers are
// strongly typed (rclcpp::Publisher<T>), so we keep the base handle and cast
// back to the concrete type on publish (the type is recorded on advertise()).
class TypedPublisher {
 public:
  TypedPublisher() : type_(std::type_index(typeid(void))) {}

  template <typename T>
  void advertise(rclcpp::Node::SharedPtr node, const std::string& topic) {
    // Publishers are created lazily on the first publish() to a topic, so the
    // first message would otherwise be sent before any subscriber has finished
    // the discovery handshake and would be dropped. transient_local makes the
    // publisher retain the last message and deliver it to subscribers as soon
    // as they connect, so the first message is never lost.
    pub_ = node->create_publisher<T>(topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    type_ = std::type_index(typeid(T));
  }

  template <typename T>
  void publish(const T& msg) const {
    std::static_pointer_cast<rclcpp::Publisher<T>>(pub_)->publish(msg);
  }

  bool isSameType(const std::type_info& type) const { return type_ == std::type_index(type); }

 private:
  rclcpp::PublisherBase::SharedPtr pub_;
  std::type_index type_;
};

// Supplies the ROS2-specific pieces to PublisherBase.
struct Ros2Backend {
  using Handle = rclcpp::Node::SharedPtr;
  using TypedPublisher = bievr::TypedPublisher;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Odometry = nav_msgs::msg::Odometry;
  using Path = nav_msgs::msg::Path;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  explicit Ros2Backend(Handle node)
      : node_(node), tf_(std::make_shared<tf2_ros::TransformBroadcaster>(node)) {}

  template <typename M>
  void advertise(TypedPublisher& pub, const std::string& topic) {
    pub.template advertise<M>(node_, topic);
  }

  void sendTransform(const TransformStamped& transform) { tf_->sendTransform(transform); }

  Handle node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
};

using Publisher = PublisherBase<Ros2Backend>;

}  // namespace bievr
#endif  // BIEVR_LIO_ROS2_PUBLISHER_H_
