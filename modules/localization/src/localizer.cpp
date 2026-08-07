#include "bievr_localization/localizer.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "bievr_lio/log++.h"

namespace bievr {
namespace {

Eigen::Matrix4f toMatrix4f(const Transform& t) { return t.matrix().cast<float>(); }

// ICP output is a raw 4x4; neither Isometry3d nor Pose3 re-orthonormalizes, so
// the rotation block is squared up here before it can compound over cycles.
Transform toTransform(const Eigen::Matrix4f& m) {
  const Eigen::Matrix4d d = m.cast<double>();
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = Eigen::Quaterniond(Eigen::Matrix3d(d.topLeftCorner<3, 3>()))
                     .normalized()
                     .toRotationMatrix();
  iso.translation() = d.topRightCorner<3, 1>();
  return Transform(iso);
}

Localizer::Cloud::Ptr toPcl(const Pointcloud& cloud) {
  Localizer::Cloud::Ptr out(new Localizer::Cloud());
  const auto& data = cloud.data();
  out->resize(data.cols());
  for (Eigen::Index i = 0; i < data.cols(); ++i) {
    out->points[i] = Localizer::PointT(static_cast<float>(data(0, i)),
                                       static_cast<float>(data(1, i)),
                                       static_cast<float>(data(2, i)));
  }
  return out;
}

Localizer::Cloud::Ptr voxelize(const Localizer::Cloud::Ptr& cloud, double leaf) {
  if (leaf <= 0.0 || cloud->empty()) return cloud;
  Localizer::Cloud::Ptr out(new Localizer::Cloud());
  pcl::VoxelGrid<Localizer::PointT> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(cloud);
  filter.filter(*out);
  return out;
}

}  // namespace

const char* Localizer::toString(Status status) {
  switch (status) {
    case Status::NoMap: return "NO_MAP";
    case Status::WaitingForPose: return "WAITING_FOR_POSE";
    case Status::Localized: return "LOCALIZED";
    case Status::Lost: return "LOST";
  }
  return "UNKNOWN";
}

Localizer::Localizer(Config config) : config_(std::move(config)) {}

Localizer::~Localizer() {
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_ = true;
  }
  stop_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

bool Localizer::waitOrStop(std::chrono::duration<double> period) {
  std::unique_lock<std::mutex> lock(stop_mutex_);
  return stop_cv_.wait_for(lock, period, [this] { return stop_.load(); });
}

bool Localizer::start(std::string* message) {
  LoadOptions options;
  options.scan_context = config_.scan_context;
  options.voxel_size_m = config_.map_voxel_size_m;
  if (!loadPriorMap(config_.map_path, options, map_, message)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status_ = Status::NoMap;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_cloud_ = map_.cloud;
    viz_cloud_ = voxelDownsample(*map_cloud_, config_.map_viz_voxel_size_m);
    ++map_generation_;
    LOG(I, "Prior map resident: " << map_cloud_->size() << " points for ICP, "
                                  << viz_cloud_->size() << " for display.");
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status_ = Status::WaitingForPose;
  }
  if (!map_.canRelocalize()) {
    LOG(I, "Localization has no descriptor database; waiting for an initial pose.");
  }

  worker_ = std::thread(&Localizer::worker, this);
  return true;
}

void Localizer::addFrame(uint64_t stamp, const Transform& T_W_I, const Pointcloud& cloud_body) {
  // Keep only the newest frame: the worker runs far slower than the odometry and
  // an older scan is never the better one to match.
  PendingFrame frame;
  frame.stamp = stamp;
  frame.T_odom_body = T_W_I;
  frame.cloud_body = toPcl(cloud_body);

  std::lock_guard<std::mutex> lock(frame_mutex_);
  pending_ = std::move(frame);
}

void Localizer::setInitialPose(const Transform& T_map_body) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  requested_pose_ = T_map_body;
}

Localizer::Correction Localizer::correction() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return correction_;
}

Localizer::Status Localizer::status() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return status_;
}

Localizer::Stats Localizer::stats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

bool Localizer::canRelocalize() const { return map_.canRelocalize(); }

Localizer::Cloud::ConstPtr Localizer::vizCloud() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return viz_cloud_;
}

uint64_t Localizer::mapGeneration() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return map_generation_;
}

std::optional<Localizer::PendingFrame> Localizer::takeFrame() {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (!pending_) return std::nullopt;
  PendingFrame frame = std::move(*pending_);
  pending_.reset();
  return frame;
}

Transform Localizer::correctionFrom(const Transform& T_map_body, const Transform& T_odom_body) {
  return Transform(Eigen::Isometry3d(T_map_body * T_odom_body.inverse()));
}

std::optional<Transform> Localizer::relocalize(const PendingFrame& frame) {
  if (!map_.canRelocalize()) return std::nullopt;

  // Descriptors were built from clouds voxelized at this leaf; querying with a
  // differently filtered cloud degrades matching with no error anywhere.
  const Cloud::Ptr query = voxelize(frame.cloud_body, map_.meta.keyframe_filter_size_m);
  Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, query->size());
  for (size_t i = 0; i < query->size(); ++i) {
    const auto& p = query->points[i];
    points.col(i) << p.x, p.y, p.z;
  }

  ScanContext::Match match;
  {
    std::lock_guard<std::mutex> lock(sc_mutex_);
    match = map_.reloc->scan_context.query(points);
  }
  if (match.index < 0 || match.index >= static_cast<int>(map_.reloc->poses.size())) {
    LOG(I, "No descriptor match (best distance " << match.distance << ").");
    return std::nullopt;
  }

  LOG(I, "Descriptor match: keyframe " << match.index << ", distance " << match.distance
                                       << ", yaw " << match.yaw_diff_rad << " rad.");
  return map_.reloc->poses[match.index];
}

Localizer::Cloud::Ptr Localizer::cropAround(const Point& centre, double scale) const {
  Cloud::Ptr cropped(new Cloud());
  // Snapshot the pointer, scan outside the lock: the scan is long and must not
  // hold up a resident-set swap.
  Cloud::ConstPtr map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_cloud_;
  }
  if (!map) return cropped;

  const float cx = static_cast<float>(centre.x());
  const float cy = static_cast<float>(centre.y());
  const float cz = static_cast<float>(centre.z());
  const float radius = static_cast<float>(config_.crop_radius_m * scale);
  const float radius_sq = radius * radius;

  cropped->reserve(map->size() / 4);
  for (const auto& p : map->points) {
    const float dx = p.x - cx;
    const float dy = p.y - cy;
    const float dz = p.z - cz;
    if (dx * dx + dy * dy + dz * dz <= radius_sq) cropped->push_back(p);
  }
  return cropped;
}

Localizer::RefineResult Localizer::refine(const Cloud& source_odom, const Transform& guess,
                                         const Point& centre) {
  RefineResult result;
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(config_.icp_max_iterations);
  icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(config_.icp_euclidean_fitness_epsilon);
  icp.setRANSACIterations(0);

  // Crop once: only the leaf sizes and the correspondence distance change
  // between passes, not which part of the map is in play.
  const Cloud::Ptr target = cropAround(centre, 1.0);
  if (target->empty()) {
    LOG(W, "Prior map has no points within " << config_.crop_radius_m << " m of the estimate.");
    return result;
  }
  const Cloud::Ptr source(new Cloud(source_odom));
  const double base_leaf = std::max(config_.map_voxel_size_m, 0.1);
  Eigen::Matrix4f transformation = toMatrix4f(guess);

  // Coarse then fine: the coarse pass tolerates a seed several metres out, the
  // fine pass tightens it. Same two-stage shape as FAST_LIO_LOCALIZATION.
  for (const double scale : {config_.coarse_scale, 1.0}) {
    const Cloud::Ptr target_scaled = voxelize(target, base_leaf * scale);
    const Cloud::Ptr source_scaled = voxelize(source, config_.scan_voxel_size_m * scale);
    if (source_scaled->empty() || target_scaled->empty()) return result;

    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance * scale);
    icp.setInputSource(source_scaled);
    icp.setInputTarget(target_scaled);

    Cloud aligned;
    icp.align(aligned, transformation);
    if (!icp.hasConverged()) return result;
    transformation = icp.getFinalTransformation();
  }

  result.transform = toTransform(transformation);
  result.fitness = icp.getFitnessScore();
  result.converged = true;
  result.accepted = result.fitness <= config_.fitness_threshold;
  return result;
}

void Localizer::worker() {
  const auto period = std::chrono::duration<double>(1.0 / std::max(config_.correction_frequency,
                                                                   1e-3));
  while (true) {
    if (waitOrStop(period)) return;

    const auto frame = takeFrame();
    if (!frame) continue;

    // A manual pose always wins: it is the operator saying the current fix is
    // wrong, so it must not be second-guessed by the previous correction.
    std::optional<Transform> requested;
    std::optional<Transform> provisional;
    Correction current;
    Status status = Status::WaitingForPose;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      requested = std::exchange(requested_pose_, std::nullopt);
      provisional = provisional_;
      current = correction_;
      status = status_;
    }

    Transform guess;
    bool seeded = false;
    if (requested) {
      guess = correctionFrom(*requested, frame->T_odom_body);
      seeded = true;
      provisional.reset();
      LOG(I, "Seeding localization from a requested pose.");
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.relocalizations;
    } else if (current.valid && status == Status::Localized) {
      guess = current.T_map_odom;
      seeded = true;
    } else if (provisional) {
      // Rejected last cycle but converged: continuing from it walks a rough
      // seed in over a few attempts instead of retrying from cold each time.
      guess = *provisional;
      seeded = true;
    } else if (const auto keyframe = relocalize(*frame)) {
      guess = correctionFrom(*keyframe, frame->T_odom_body);
      seeded = true;
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.relocalizations;
    }
    if (!seeded) continue;

    // ICP source is the scan in the odom frame and the target is the map frame,
    // so what comes out is T_map_odom itself.
    Cloud::Ptr source_odom(new Cloud());
    pcl::transformPointCloud(*frame->cloud_body, *source_odom, toMatrix4f(frame->T_odom_body));

    const Point centre = (guess * frame->T_odom_body).translation();
    const RefineResult result = refine(*source_odom, guess, centre);

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.attempts;
      stats_.last_fitness = result.fitness;
      if (result.accepted) {
        ++stats_.accepted;
      } else {
        ++stats_.rejected;
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result.accepted) {
      correction_.T_map_odom = result.transform;
      correction_.stamp = frame->stamp;
      correction_.valid = true;
      status_ = Status::Localized;
      consecutive_failures_ = 0;
      provisional_.reset();
      continue;
    }

    // Keep reporting the last good correction so TF never breaks, but stop
    // trusting it once the failures pile up and go looking for a fresh seed.
    provisional_ = result.converged ? std::optional<Transform>(result.transform) : std::nullopt;
    ++consecutive_failures_;
    if (consecutive_failures_ >= config_.max_consecutive_failures) {
      if (status_ == Status::Localized) status_ = Status::Lost;
      LOG(W, "Localization rejected " << consecutive_failures_ << " matches (fitness "
                                      << result.fitness << "); seeking a new seed.");
      provisional_.reset();
      consecutive_failures_ = 0;
    }
  }
}

}  // namespace bievr
