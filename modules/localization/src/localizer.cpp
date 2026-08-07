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
  options.tile_size_m = config_.tile_size_m;
  options.overview_voxel_size_m = config_.map_viz_voxel_size_m;
  if (!loadPriorMap(config_.map_path, options, map_, message)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status_ = Status::NoMap;
    return false;
  }
  ring_ = std::max(1, static_cast<int>(std::ceil(config_.crop_radius_m /
                                                 std::max(config_.tile_size_m, 1.0))));

  Cloud::Ptr viz;
  if (map_.tiled()) {
    // Nothing but the overview is resident yet, and that is correct: tiles are
    // paged in around the first fix, and until there is one there is no centre
    // to page around. Scan Context relocalization never touches the cloud.
    viz.reset(new Cloud(*map_.tiles->overview()));
    LOG(I, "Prior map tiled: " << map_.tiles->tiles().size() << " tiles, "
                               << map_.tiles->numPoints() << " points, resident ring " << ring_
                               << " (" << (2 * ring_ + 1) << "x" << (2 * ring_ + 1) << "), "
                               << viz->size() << " points for display.");
  } else {
    viz = voxelDownsample(*map_.cloud, config_.map_viz_voxel_size_m);
    rebuildTargets(map_.cloud);
    LOG(I, "Prior map resident whole: " << map_.cloud->size() << " points for ICP, "
                                        << viz->size() << " for display.");
  }
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    viz_cloud_ = viz;
    ++map_generation_;
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

void Localizer::rebuildTargets(const Cloud::Ptr& fine) {
  target_fine_ = fine;
  // Tiles are already stored at map_voxel_size_m, so the fine target needs no
  // further filtering; only the coarse pass gets its own copy.
  const double base_leaf = std::max(config_.map_voxel_size_m, 0.1);
  target_coarse_ = voxelDownsample(*fine, base_leaf * config_.coarse_scale);

  // The trees are the point of all this: built once per resident set, then
  // handed to every ICP call that follows with force_no_recompute.
  tree_fine_.reset(new pcl::search::KdTree<PointT>());
  tree_fine_->setInputCloud(target_fine_);
  tree_coarse_.reset(new pcl::search::KdTree<PointT>());
  tree_coarse_->setInputCloud(target_coarse_);
}

bool Localizer::updateResident(const Point& centre) {
  if (!map_.tiled()) return target_fine_ && !target_fine_->empty();

  const TileKey key = map_.tiles->keyAt(centre.x(), centre.y());
  if (resident_centre_ && *resident_centre_ == key) {
    return target_fine_ && !target_fine_->empty();
  }
  resident_centre_ = key;

  const std::vector<TileKey> ring = map_.tiles->ringAround(key, ring_);
  bool changed = false;
  for (const TileKey& tile : ring) {
    if (resident_.count(tile)) continue;
    if (Cloud::Ptr cloud = map_.tiles->load(tile)) {
      resident_.emplace(tile, cloud);
      changed = true;
    }
  }

  // Evict one ring further out than we load, so a pose sitting on a tile
  // boundary does not page the same tiles in and out every cycle.
  for (auto it = resident_.begin(); it != resident_.end();) {
    const int64_t distance = std::max(std::abs(it->first.x - key.x), std::abs(it->first.y - key.y));
    if (distance > ring_ + 1) {
      it = resident_.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }

  // Crossing into a tile whose whole ring is already resident costs nothing:
  // the targets and their trees stay exactly as they were.
  if (!changed && target_fine_) return !target_fine_->empty();

  size_t total = 0;
  for (const TileKey& tile : ring) {
    const auto it = resident_.find(tile);
    if (it != resident_.end()) total += it->second->size();
  }
  Cloud::Ptr fine(new Cloud());
  fine->reserve(total);
  for (const TileKey& tile : ring) {
    const auto it = resident_.find(tile);
    if (it != resident_.end()) *fine += *it->second;
  }
  if (fine->empty()) {
    LOG(W, "No prior map around (" << centre.x() << ", " << centre.y() << ").");
    target_fine_ = fine;
    return false;
  }

  rebuildTargets(fine);
  LOG(I, "Prior map tiles at (" << key.x << ", " << key.y << "): " << ring.size()
                                << " in the ring, " << resident_.size() << " held, "
                                << fine->size() << " points in the ICP target.");
  return true;
}

Localizer::RefineResult Localizer::refine(const Cloud& source_odom, const Transform& guess,
                                         const Point& centre) {
  RefineResult result;
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(config_.icp_max_iterations);
  icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(config_.icp_euclidean_fitness_epsilon);
  icp.setRANSACIterations(0);

  // Page in whatever tiles the estimate sits on. Both ICP passes then run
  // against the cached targets and their prebuilt trees.
  if (!updateResident(centre)) return result;

  const Cloud::Ptr source(new Cloud(source_odom));
  Eigen::Matrix4f transformation = toMatrix4f(guess);

  // Coarse then fine: the coarse pass tolerates a seed several metres out, the
  // fine pass tightens it. Same two-stage shape as FAST_LIO_LOCALIZATION.
  const std::pair<double, bool> passes[] = {{config_.coarse_scale, true}, {1.0, false}};
  for (const auto& [scale, coarse] : passes) {
    const Cloud::Ptr target = coarse ? target_coarse_ : target_fine_;
    const auto& tree = coarse ? tree_coarse_ : tree_fine_;
    const Cloud::Ptr source_scaled = voxelize(source, config_.scan_voxel_size_m * scale);
    if (source_scaled->empty() || !target || target->empty()) return result;

    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance * scale);
    icp.setInputSource(source_scaled);
    icp.setInputTarget(target);
    // Order does not matter, but this must come after every setInputTarget:
    // force_no_recompute is what stops initCompute rebuilding the tree we just
    // spent a resident-set change building.
    icp.setSearchMethodTarget(tree, /*force_no_recompute=*/true);

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
