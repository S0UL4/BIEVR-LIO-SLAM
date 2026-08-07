#include "bievr_map_io/cloud_utils.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <Eigen/Core>

namespace bievr {
namespace {

// Exact key comparison, so the hash only has to spread -- collisions are
// resolved by operator== and never merge two voxels.
struct VoxelKey {
  int64_t x = 0, y = 0, z = 0;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& k) const {
    return static_cast<size_t>((k.x * 73856093) ^ (k.y * 19349663) ^ (k.z * 83492791));
  }
};

struct VoxelAccum {
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  uint32_t count = 0;
};

}  // namespace

struct VoxelAccumulator::Impl {
  double inv_leaf = 0.0;
  bool passthrough = false;  // leaf <= 0: keep every point
  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
  MapCloud::Ptr verbatim;
};

VoxelAccumulator::VoxelAccumulator(double leaf_m) : impl_(new Impl()) {
  impl_->passthrough = leaf_m <= 0.0;
  impl_->inv_leaf = impl_->passthrough ? 0.0 : 1.0 / leaf_m;
  if (impl_->passthrough) impl_->verbatim.reset(new MapCloud());
}

VoxelAccumulator::~VoxelAccumulator() = default;

void VoxelAccumulator::reserve(size_t voxels) {
  if (!impl_->passthrough) impl_->voxels.reserve(voxels);
}

size_t VoxelAccumulator::size() const {
  return impl_->passthrough ? impl_->verbatim->size() : impl_->voxels.size();
}

void VoxelAccumulator::add(const MapCloud& cloud) {
  if (impl_->passthrough) {
    *impl_->verbatim += cloud;
    return;
  }
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    // std::floor, never a cast: truncation toward zero makes the origin voxel
    // double width.
    VoxelKey key;
    key.x = static_cast<int64_t>(std::floor(p.x * impl_->inv_leaf));
    key.y = static_cast<int64_t>(std::floor(p.y * impl_->inv_leaf));
    key.z = static_cast<int64_t>(std::floor(p.z * impl_->inv_leaf));

    VoxelAccum& accum = impl_->voxels[key];
    accum.sum += p.getVector3fMap();
    ++accum.count;
  }
}

MapCloud::Ptr VoxelAccumulator::take() {
  if (impl_->passthrough) {
    MapCloud::Ptr out = impl_->verbatim;
    impl_->verbatim.reset(new MapCloud());
    return out;
  }

  MapCloud::Ptr out(new MapCloud());
  out->reserve(impl_->voxels.size());
  for (const auto& entry : impl_->voxels) {
    const Eigen::Vector3f centroid = entry.second.sum / static_cast<float>(entry.second.count);
    out->push_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  impl_->voxels.clear();
  return out;
}

MapCloud::Ptr voxelDownsample(const MapCloud& cloud, double leaf_m) {
  VoxelAccumulator accumulator(leaf_m);
  accumulator.reserve(cloud.size() / 8);  // a map is mostly repeat passes
  accumulator.add(cloud);
  return accumulator.take();
}

}  // namespace bievr
