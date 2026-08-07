#ifndef BIEVR_MAP_IO_PRIOR_MAP_H_
#define BIEVR_MAP_IO_PRIOR_MAP_H_

// Reads and writes the prior map that localization runs against.
//
// The only thing localization strictly needs is a point cloud, so a bare .pcd
// from any SLAM works. Our own mapping run additionally saves a Scan Context
// database and the keyframe poses it is indexed by; when those are present
// localization can relocalize by itself instead of waiting for an initial pose.
// That makes the bundle an optional accelerator on top of a PCD, never a
// requirement.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "bievr_lio/common.h"
#include "bievr_scancontext/scan_context.h"

namespace bievr {

using MapCloud = pcl::PointCloud<pcl::PointXYZ>;

// How the map was built. Everything here has a usable default, because a bare
// PCD carries none of it.
struct MapMeta {
  std::string frame = "odom";
  size_t num_keyframes = 0;
  size_t num_points = 0;
  double map_save_resolution_m = 0.1;
  // Voxel leaf the keyframe clouds were filtered at before they reached Scan
  // Context. Query clouds must be filtered the same way or descriptors are not
  // comparable, which degrades matching silently.
  double keyframe_filter_size_m = 0.4;
  ScanContext::Config scan_context;
};

// The descriptor database and the poses it is indexed by. Index i of `poses`,
// `stamps` and descriptor i all refer to the same keyframe -- that alignment is
// what turns a Scan Context match into a pose.
struct Relocalization {
  ScanContext scan_context;
  std::vector<Transform> poses;
  std::vector<uint64_t> stamps;
};

struct PriorMap {
  MapCloud::Ptr cloud;
  std::optional<Relocalization> reloc;
  MapMeta meta;

  bool canRelocalize() const { return reloc.has_value(); }
};

struct LoadOptions {
  // Scan Context settings used for querying. Only the non-geometric fields take
  // effect -- ScanContext::load adopts rings/sectors/radius/height from the file,
  // since descriptors are only comparable under the geometry that built them.
  ScanContext::Config scan_context;
  // Voxel leaf applied to the loaded cloud, <= 0 keeps it as stored. A foreign
  // PCD can have any density; normalising it here keeps ICP predictable.
  double voxel_size_m = 0.0;
};

// Voxel downsample, one centroid per occupied voxel. Used instead of
// pcl::VoxelGrid because that one indexes its grid with an int32 and, when
// extent/leaf overflows it, silently returns the cloud *unfiltered* -- which a
// 1 km map at a 0.3 m leaf does. Keys here are per-axis int64, so only the
// occupied voxels cost anything and the extent is irrelevant.
MapCloud::Ptr voxelDownsample(const MapCloud& cloud, double leaf_m);

// `path` is either a .pcd file (cloud only) or a bundle directory. A directory
// whose scan_context.bin / poses_tum.txt are missing or inconsistent still
// loads, cloud only, with the reason reported through `message`. Fails only when
// the cloud itself cannot be read.
bool loadPriorMap(const std::string& path, const LoadOptions& options, PriorMap& map,
                  std::string* message = nullptr);

// Writes cloud.pcd, poses_tum.txt, scan_context.bin and meta.yaml into `dir`.
// `descriptors` is a snapshot so the caller can take it under its own lock. The
// directory is created only once the inputs are known to be writable.
bool saveMapBundle(const std::string& dir, const MapCloud& cloud,
                   const std::vector<Transform>& poses, const std::vector<uint64_t>& stamps,
                   const ScanContext::Archive& descriptors, const MapMeta& meta,
                   std::string* message = nullptr);

}  // namespace bievr

#endif  // BIEVR_MAP_IO_PRIOR_MAP_H_
