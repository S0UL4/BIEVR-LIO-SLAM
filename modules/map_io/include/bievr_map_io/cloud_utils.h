#ifndef BIEVR_MAP_IO_CLOUD_UTILS_H_
#define BIEVR_MAP_IO_CLOUD_UTILS_H_

// Cloud primitives shared by the prior map and the tile cache.

#include <cstddef>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace bievr {

using MapCloud = pcl::PointCloud<pcl::PointXYZ>;

// Accumulates a voxel downsample across several clouds, so a map that is read in
// pieces never has to be held whole to be filtered.
//
// Used instead of pcl::VoxelGrid because that one indexes its grid with an int32
// and, when extent/leaf overflows it, silently returns the cloud *unfiltered* --
// which a 1 km map at a 0.3 m leaf does. Keys here are per-axis int64, so only
// the occupied voxels cost anything and the extent is irrelevant.
class VoxelAccumulator {
 public:
  explicit VoxelAccumulator(double leaf_m);
  ~VoxelAccumulator();

  VoxelAccumulator(const VoxelAccumulator&) = delete;
  VoxelAccumulator& operator=(const VoxelAccumulator&) = delete;

  void add(const MapCloud& cloud);
  void reserve(size_t voxels);
  size_t size() const;  // occupied voxels so far

  // One centroid per occupied voxel. Clears the accumulator.
  MapCloud::Ptr take();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// One-shot form of the above. A leaf <= 0 copies the cloud unchanged.
MapCloud::Ptr voxelDownsample(const MapCloud& cloud, double leaf_m);

}  // namespace bievr

#endif  // BIEVR_MAP_IO_CLOUD_UTILS_H_
