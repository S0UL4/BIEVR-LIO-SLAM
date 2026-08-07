#ifndef BIEVR_MAP_IO_TILE_MAP_H_
#define BIEVR_MAP_IO_TILE_MAP_H_

// A prior map split into fixed-size 2D tiles on disk, so a consumer can hold the
// ring of tiles around its current fix instead of the whole cloud.
//
// The cache is built once beside the source (`<cloud.pcd>.tiles/`) and reused.
// index.yaml records both what it was built from (path, size, mtime) and how
// (tile size, voxel leaves), so a changed source or a changed parameter rebuilds
// it and nothing else does. It is assembled in a temporary directory and moved
// into place, so an interrupted build never replaces a good cache.
//
// Tiles are voxelized as they are written, which is what makes startup cheap:
// what the cache holds is already at the resolution ICP wants, and no run ever
// re-filters the source again.

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "bievr_map_io/cloud_utils.h"

namespace bievr {

// 2D tile index. Only x/y are tiled: maps are wide and thin, and splitting z
// would evict the ground out from under the sensor.
struct TileKey {
  int64_t x = 0;
  int64_t y = 0;
  bool operator==(const TileKey& other) const { return x == other.x && y == other.y; }
};

struct TileKeyHash {
  size_t operator()(const TileKey& k) const {
    return static_cast<size_t>((k.x * 73856093) ^ (k.y * 19349663));
  }
};

struct TileInfo {
  TileKey key;
  size_t num_points = 0;
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();
};

class TileMap {
 public:
  struct Config {
    double tile_size_m = 100.0;
    double voxel_size_m = 0.3;           // leaf the tiles are written at
    double overview_voxel_size_m = 0.4;  // leaf of the single cloud kept for display
  };

  // Opens `<source>.tiles/`, building it first when it is missing, was built
  // from a different file, or was built with different parameters. `source` is a
  // .pcd path. Returns false with a reason when no cache could be produced --
  // an unwritable directory, say -- which the caller is expected to treat as
  // "fall back to loading the cloud", not as an error.
  bool open(const std::string& source, const Config& config, std::string* message = nullptr);

  const Config& config() const { return config_; }
  const std::vector<TileInfo>& tiles() const { return tiles_; }
  size_t numPoints() const { return num_points_; }
  const Eigen::Vector3f& min() const { return min_; }
  const Eigen::Vector3f& max() const { return max_; }

  // Coarse copy of the whole map, for display and for giving an operator
  // something to click on before the first fix. Never null after open().
  MapCloud::ConstPtr overview() const { return overview_; }

  // The tile containing (x, y).
  TileKey keyAt(double x, double y) const;
  // Occupied tiles within `ring` of `centre` (Chebyshev distance, so `ring` 2 is
  // the 5x5 block).
  std::vector<TileKey> ringAround(const TileKey& centre, int ring) const;
  bool has(const TileKey& key) const { return index_.count(key) != 0; }

  // Reads one tile. Null when the key is not occupied or the file is unreadable.
  MapCloud::Ptr load(const TileKey& key) const;

 private:
  bool readIndex(const std::string& dir, const std::string& source, std::string* message);
  bool build(const std::string& dir, const std::string& source, std::string* message);
  bool writeIndex(const std::string& dir, const std::string& source) const;
  // Regenerates overview.pcd from the tiles. The display leaf is deliberately
  // not part of the cache key: changing it must not cost a pass over the source.
  bool rebuildOverview(const std::string& dir, const std::string& source);
  std::string tilePath(const std::string& dir, const TileKey& key) const;

  Config config_;
  std::string dir_;
  std::vector<TileInfo> tiles_;
  std::unordered_map<TileKey, size_t, TileKeyHash> index_;  // key -> index into tiles_
  size_t num_points_ = 0;
  Eigen::Vector3f min_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_ = Eigen::Vector3f::Zero();
  MapCloud::Ptr overview_;
  double cached_overview_leaf_ = 0.0;  // what the index on disk says overview.pcd is
};

}  // namespace bievr

#endif  // BIEVR_MAP_IO_TILE_MAP_H_
