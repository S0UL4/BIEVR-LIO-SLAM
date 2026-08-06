#include "bievr_lio/map_accumulator.h"

#include <utility>
#include <vector>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

MapAccumulator::MapAccumulator(double resolution, size_t max_queue)
    : inv_resolution_(1.0 / resolution), max_queue_(max_queue) {
  worker_ = std::thread(&MapAccumulator::run, this);
}

MapAccumulator::~MapAccumulator() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void MapAccumulator::add(Pointcloud registered) {
  if (registered.empty()) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(std::move(registered));
    while (queue_.size() > max_queue_) {
      queue_.pop_front();
      ++dropped_;
      LOG_FIRST(W, 1, "Map accumulation is falling behind; dropping scans from the saved map. "
                          << "Increase map_save.resolution_m to make it cheaper.");
    }
  }
  queue_cv_.notify_one();
}

void MapAccumulator::run() {
  while (true) {
    Pointcloud cloud;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      // Drain whatever is still queued before shutting down.
      if (queue_.empty()) return;
      cloud = std::move(queue_.front());
      queue_.pop_front();
    }

    // Hash outside the map lock so a concurrent save() only ever waits for the
    // merge below, not for the whole scan to be processed.
    std::vector<std::pair<size_t, Point>> hashed;
    hashed.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
      const Point p = cloud[i];
      const Eigen::Vector3i voxel = (p * inv_resolution_).array().floor().cast<int>();
      hashed.emplace_back(hashIndexVoxel(voxel), p);
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    voxels_.reserve(voxels_.size() + hashed.size());
    for (const auto& [hash, point] : hashed) {
      // Keep the first point seen in a voxel and skip the rest, so revisiting
      // an area does not keep growing the cloud.
      voxels_.try_emplace(hash, point);
    }
  }
}

bool MapAccumulator::save(const std::string& path, size_t* num_points) const {
  Pointcloud cloud;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (voxels_.empty()) {
      LOG(E, "Cannot save map: nothing has been accumulated yet.");
      return false;
    }
    cloud.resize(voxels_.size());
    size_t i = 0;
    for (const auto& [hash, point] : voxels_) {
      cloud[i++] = point;
    }
  }

  if (!savePointcloudPCD(cloud, path)) return false;
  if (num_points != nullptr) *num_points = cloud.size();
  LOG(I, "Saved map with " << cloud.size() << " points to '" << path << "'.");
  return true;
}

size_t MapAccumulator::size() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return voxels_.size();
}

size_t MapAccumulator::droppedScans() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return dropped_;
}

}  // namespace bievr
