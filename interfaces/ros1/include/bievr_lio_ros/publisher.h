#ifndef LIO_ROS_PUBLISHER_H_
#define LIO_ROS_PUBLISHER_H_

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <typeindex>
#include <typeinfo>

// publisher_base.h holds the shared publish logic and transitively includes
// bievr_ros_common/conversions.h (which pulls in the ROS message headers).
#include "bievr_ros_common/publisher_base.h"

namespace bievr {

// Type-erased wrapper around a ros::Publisher. The concrete message type is
// recorded on advertise() so publish() can be checked against it.
class TypedPublisher {
 public:
  TypedPublisher() : type_(std::type_index(typeid(void))) {}

  template <typename T>
  void advertise(ros::NodeHandle& nh, const std::string& topic) {
    // Publishers are created lazily on the first publish() to a topic, so the
    // first message would otherwise be sent before any subscriber has finished
    // connecting and would be dropped. Latching makes the publisher retain the
    // last message and deliver it to subscribers as soon as they connect, so
    // the first message is never lost.
    pub_ = nh.advertise<T>(topic, 10, /*latch=*/true);
    type_ = std::type_index(typeid(T));
  }

  template <typename T>
  void publish(const T& msg) const {
    pub_.publish(msg);
  }

  bool isSameType(const std::type_info& type) const { return type_ == std::type_index(type); }

 private:
  ros::Publisher pub_;
  std::type_index type_;
};

// Supplies the ROS1-specific pieces to PublisherBase.
struct RosBackend {
  using Handle = ros::NodeHandle;
  using TypedPublisher = bievr::TypedPublisher;
  using PointCloud2 = sensor_msgs::PointCloud2;
  using Odometry = nav_msgs::Odometry;
  using Path = nav_msgs::Path;
  using Vector3Stamped = geometry_msgs::Vector3Stamped;
  using TransformStamped = geometry_msgs::TransformStamped;

  explicit RosBackend(Handle nh) : nh_(nh) {}

  template <typename M>
  void advertise(TypedPublisher& pub, const std::string& topic) {
    pub.template advertise<M>(nh_, topic);
  }

  void sendTransform(const TransformStamped& transform) { tf_.sendTransform(transform); }

  Handle nh_;
  tf::TransformBroadcaster tf_;
};

using Publisher = PublisherBase<RosBackend>;

}  // namespace bievr
#endif  // LIO_ROS_PUBLISHER_H_
