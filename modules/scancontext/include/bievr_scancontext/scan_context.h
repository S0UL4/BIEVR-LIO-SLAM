#ifndef BIEVR_SCANCONTEXT_SCAN_CONTEXT_H_
#define BIEVR_SCANCONTEXT_SCAN_CONTEXT_H_

// Scan Context (Kim & Kim, IROS 18) place descriptor.
//
// Ported from SC-A-LOAM's SCManager. Depends on Eigen only -- no PCL, no ROS --
// so both the mapping side (loop detection) and the localization side
// (relocalization against a saved database) can link it.

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bievr_scancontext/KDTreeVectorOfVectorsAdaptor.h"

namespace bievr {

// Body-frame (sensor-centric) points, 3xN. Descriptors are not comparable
// across different sensor origins.
using ScanContextCloud = Eigen::Ref<const Eigen::Matrix<double, 3, Eigen::Dynamic>>;

class ScanContext {
 public:
  struct Config {
    int num_rings = 20;      // radial bins
    int num_sectors = 60;    // azimuthal bins
    double max_radius = 80.0;  // points beyond this are ignored [m]
    // Added to every point's z so bins hold height above ground: set it to the
    // sensor origin's height above the ground plane. Too small and occupied
    // bins go negative, colliding with the zero that marks an empty bin.
    double lidar_height = 2.0;
    int num_exclude_recent = 30;  // keyframes before the query that cannot close a loop
    int num_candidates = 3;       // kNN candidates pulled from the ring-key tree
    double search_ratio = 0.1;    // fraction of shifts searched around the sector-key guess
    double dist_threshold = 0.2;  // accept a match below this cosine distance
    int tree_making_period = 30;  // rebuild the kd-tree every N queries
  };

  struct Match {
    int index = -1;  // -1 means no match
    float yaw_diff_rad = 0.0f;
    double distance = 1.0;  // best candidate's distance, match or not
  };

  // No `Config config = {}` default argument: a nested class's member
  // initializers are not usable in the enclosing class's own declarations.
  ScanContext() = default;
  explicit ScanContext(Config config);

  // Appends a keyframe to the database.
  void add(const ScanContextCloud& cloud_body);

  // Matches the newest entry against the older ones, skipping the most recent
  // `num_exclude_recent`. Mapping-side loop detection.
  Match detectLoop();

  // Matches an external scan against the whole database without adding it.
  // Localization-side relocalization.
  Match query(const ScanContextCloud& cloud_body);

  bool save(const std::string& path) const;
  bool load(const std::string& path);

  // Everything save() writes. Snapshot with archive() under whatever lock guards
  // this object, then write with saveArchive() outside it, so a slow disk never
  // stalls add()/detectLoop().
  struct Archive {
    Config config;
    std::vector<Eigen::MatrixXd> descriptors;
  };
  Archive archive() const { return Archive{config_, descriptors_}; }
  static bool saveArchive(const Archive& archive, const std::string& path);

  // Drops everything past `count`, for callers reconciling the database with a
  // pose list it must stay index-aligned with. A no-op when already smaller.
  void truncate(size_t count);

  size_t size() const { return descriptors_.size(); }
  const Config& config() const { return config_; }

  Eigen::MatrixXd makeDescriptor(const ScanContextCloud& cloud) const;

 private:
  using KeyMat = std::vector<std::vector<float>>;
  using KeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

  Eigen::MatrixXd ringKey(const Eigen::MatrixXd& desc) const;
  Eigen::MatrixXd sectorKey(const Eigen::MatrixXd& desc) const;

  double directDistance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const;
  int alignBySectorKey(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const;
  // Best cosine distance over the searched shifts, and the winning shift.
  std::pair<double, int> distance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const;

  // kNN over `tree` (indexing the first `num_searchable` entries), then exact
  // scoring of the candidates against `desc`.
  Match search(KeyTree& tree, const std::vector<float>& key, const Eigen::MatrixXd& desc,
               size_t num_searchable) const;

  void append(Eigen::MatrixXd desc);

  Config config_;
  std::vector<Eigen::MatrixXd> descriptors_;
  KeyMat ring_keys_;

  // detectLoop()'s tree, rebuilt periodically over all but the recent entries.
  KeyMat search_keys_;
  std::unique_ptr<KeyTree> tree_;
  int tree_counter_ = 0;

  // query()'s tree, over every entry. Rebuilt when the database grows.
  KeyMat batch_keys_;
  std::unique_ptr<KeyTree> batch_tree_;
  size_t batch_size_ = 0;
};

}  // namespace bievr

#endif  // BIEVR_SCANCONTEXT_SCAN_CONTEXT_H_
