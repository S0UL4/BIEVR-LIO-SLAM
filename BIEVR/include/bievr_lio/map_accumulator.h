#ifndef BIEVR_LIO_MAP_ACCUMULATOR_H_
#define BIEVR_LIO_MAP_ACCUMULATOR_H_

// Accumulates the registered scans into a voxel-downsampled world-frame cloud
// that can be written out on demand (see Pipeline::saveMap).
//
// The voxel hashing runs on its own thread so it never lengthens the odometry's
// per-frame critical path: processFrame hands the cloud over and returns
// immediately. That costs one cloud copy per scan, which is a flat memcpy and
// far cheaper than hashing every point inline.

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "bievr_lio/common.h"
#include "unordered_dense/unordered_dense.h"

namespace bievr {

class MapAccumulator {
 public:
  // `resolution` is the voxel size the cloud is thinned to [m]: one point is
  // kept per occupied voxel, so memory scales with the size of the scene rather
  // than with the length of the run. `max_queue` bounds the handoff backlog --
  // once the worker falls that far behind the oldest pending scans are dropped,
  // because a gap in the saved map beats growing memory without limit.
  explicit MapAccumulator(double resolution, size_t max_queue = 8);
  ~MapAccumulator();

  MapAccumulator(const MapAccumulator&) = delete;
  MapAccumulator& operator=(const MapAccumulator&) = delete;

  // Hands a world-frame cloud to the worker and returns without touching the map.
  void add(Pointcloud registered);

  // Snapshots the map and writes it as a binary PCD. Returns false if nothing
  // has been accumulated yet or the file cannot be written.
  bool save(const std::string& path, size_t* num_points = nullptr) const;

  size_t size() const;
  size_t droppedScans() const;

 private:
  void run();

  double inv_resolution_;
  size_t max_queue_;

  mutable std::mutex map_mutex_;
  ankerl::unordered_dense::map<size_t, Point> voxels_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Pointcloud> queue_;
  size_t dropped_{0};
  bool stop_{false};

  std::thread worker_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_MAP_ACCUMULATOR_H_
