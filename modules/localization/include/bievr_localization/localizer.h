#ifndef BIEVR_LOCALIZATION_LOCALIZER_H_
#define BIEVR_LOCALIZATION_LOCALIZER_H_

// Tracks a prior map, downstream of the odometry and never feeding back into it.
// Ported in shape from FAST_LIO_LOCALIZATION's global_localization.py.
//
// The trick that keeps this cheap: ICP runs with the scan already in the odom
// frame against a map-frame prior, so its output *is* T_map_odom -- no pose
// composition, and the previous correction is a valid seed for the next one.
//
// Feed it with Pipeline::addFrameObserver. addFrame() only copies and returns;
// cropping and ICP happen on a worker at correction_frequency.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "bievr_lio/common.h"
#include "bievr_map_io/prior_map.h"
#include "bievr_scancontext/scan_context.h"

namespace bievr {

class Localizer {
 public:
  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;

  struct Config {
    // A .pcd file or a bundle directory. Only a bundle can relocalize by itself.
    std::string map_path;
    std::string map_frame = "map";

    double map_voxel_size_m = 0.0;    // leaf the prior map is held at, <=0 keeps it
    // Coarser leaf for the published copy. Display needs nowhere near the
    // density ICP does, and this is the difference between a message RViz can
    // draw and one it chokes on.
    double map_viz_voxel_size_m = 0.4;
    double scan_voxel_size_m = 0.5;   // leaf applied to the ICP source scan
    // Side of one prior-map tile. The ICP target is the ring of tiles around the
    // fix, so this and crop_radius_m together decide how much map is resident.
    // <=0 disables tiling and holds the whole cloud.
    double tile_size_m = 100.0;
    // How much map to keep around the fix. Rounded *up* to whole tiles:
    // ring = ceil(crop_radius_m / tile_size_m), which at 150/100 is a 5x5 block,
    // not 3x3 -- too small a ring truncates the ICP target near a tile edge and
    // shows up only as unexplained fitness spikes.
    double crop_radius_m = 150.0;
    double correction_frequency = 0.5;  // Hz

    // Coarse-to-fine ICP. The coarse pass reruns at `coarse_scale` times the
    // leaf sizes and correspondence distance, which is what lets a metres-off
    // seed converge before the fine pass tightens it. Sized for initialization
    // rather than tracking: a few degrees of yaw error displaces points at the
    // far edge of the crop by many metres.
    double coarse_scale = 10.0;
    double icp_max_correspondence_distance = 1.0;  // fine pass [m]
    int icp_max_iterations = 20;
    double icp_transformation_epsilon = 1e-6;
    double icp_euclidean_fitness_epsilon = 1e-6;
    // PCL's getFitnessScore() is the MEAN SQUARED correspondence distance in m^2
    // -- not Open3D's inlier fraction, so the reference's 0.95 does not carry
    // over. Reject above this. Needs tuning per environment.
    double fitness_threshold = 1.0;

    // Scan Context query settings. Geometry always comes from the bundle.
    ScanContext::Config scan_context;
    // Seed ICP with the yaw the descriptor match reports. Both signs are tried,
    // as in the loop closer, because the sign convention is unverified.
    bool use_sc_yaw_guess = true;

    // Consecutive ICP rejections before the fix is abandoned and a new seed is
    // sought. The last good correction keeps being reported meanwhile.
    int max_consecutive_failures = 5;
  };

  enum class Status {
    NoMap,           // the prior map could not be loaded
    WaitingForPose,  // no seed yet: no descriptor match, and no /initialpose
    Localized,       // tracking
    Lost,            // was tracking, ICP has failed max_consecutive_failures times
  };
  static const char* toString(Status status);

  struct Correction {
    Transform T_map_odom;
    uint64_t stamp = 0;   // odometry stamp the correction was computed against
    bool valid = false;   // false until the first accepted ICP
  };

  struct Stats {
    size_t attempts = 0;
    size_t accepted = 0;
    size_t rejected = 0;
    size_t relocalizations = 0;  // times a fresh seed was taken
    double last_fitness = 0.0;
  };

  explicit Localizer(Config config);
  ~Localizer();

  Localizer(const Localizer&) = delete;
  Localizer& operator=(const Localizer&) = delete;

  // Reads the prior map and starts the worker. Must succeed before addFrame does
  // anything useful; on failure the status stays NoMap.
  bool start(std::string* message = nullptr);

  // Odometry-thread entry point. Returns immediately.
  void addFrame(uint64_t stamp, const Transform& T_W_I, const Pointcloud& cloud_body);

  // Manual seed in the map frame (RViz /initialpose). Accepted at any time and
  // takes precedence over a descriptor match, so it doubles as the recovery
  // handle when tracking has drifted onto the wrong place.
  void setInitialPose(const Transform& T_map_body);

  Correction correction() const;
  Status status() const;
  Stats stats() const;

  // True when the loaded map carries a descriptor database.
  bool canRelocalize() const;

  // The prior map in the map frame at display resolution, and a counter that
  // changes whenever the resident map does. Publishers republish on a change
  // rather than on a timer -- the map is far too heavy to send periodically.
  // Null until start().
  Cloud::ConstPtr vizCloud() const;
  uint64_t mapGeneration() const;

 private:
  struct PendingFrame {
    uint64_t stamp = 0;
    Transform T_odom_body;
    Cloud::Ptr cloud_body;
  };

  void worker();
  bool waitOrStop(std::chrono::duration<double> period);

  // Newest frame handed over by the odometry, or nullopt if none is new.
  std::optional<PendingFrame> takeFrame();

  // T_map_odom from a body-frame seed and the odometry pose it pairs with.
  static Transform correctionFrom(const Transform& T_map_body, const Transform& T_odom_body);

  // Descriptor match against the prior map, or nullopt when unavailable.
  std::optional<Transform> relocalize(const PendingFrame& frame);

  // Pages the ring of tiles around `centre` in and everything past ring + 1 out,
  // rebuilding the ICP targets and their search trees only when the resident set
  // actually changed. False when there is no map under `centre` to match against.
  // Worker thread only.
  bool updateResident(const Point& centre);
  void rebuildTargets(const Cloud::Ptr& fine);

  struct RefineResult {
    Transform transform;
    double fitness = 0.0;
    bool converged = false;
    bool accepted = false;  // converged and passed the fitness test
  };
  // Refines `guess` (a T_map_odom) against the prior map. A converged result is
  // returned even when rejected, so the next cycle can carry on from it.
  RefineResult refine(const Cloud& source_odom, const Transform& guess, const Point& centre);

  Config config_;
  PriorMap map_;

  // The display copy and a counter bumped whenever it changes. Guarded because
  // the publisher reads it off its own thread.
  mutable std::mutex map_mutex_;
  Cloud::Ptr viz_cloud_;
  uint64_t map_generation_ = 0;

  // Resident tiles and the ICP targets built from them. Worker thread only.
  // Both targets and both trees are rebuilt together and only when a tile is
  // paged in or out, which is what makes the search trees worth keeping: they
  // survive every correction cycle that stays inside the same tile.
  int ring_ = 1;
  std::unordered_map<TileKey, Cloud::Ptr, TileKeyHash> resident_;
  std::optional<TileKey> resident_centre_;
  Cloud::Ptr target_fine_;
  Cloud::Ptr target_coarse_;
  pcl::search::KdTree<PointT>::Ptr tree_fine_;
  pcl::search::KdTree<PointT>::Ptr tree_coarse_;

  std::atomic<bool> stop_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  mutable std::mutex frame_mutex_;
  std::optional<PendingFrame> pending_;

  // Scan Context's query() lazily rebuilds its tree, so it is not const.
  mutable std::mutex sc_mutex_;

  mutable std::mutex state_mutex_;
  Correction correction_;
  Status status_ = Status::NoMap;
  std::optional<Transform> requested_pose_;  // pending /initialpose
  // Last converged-but-rejected estimate. Carried into the next cycle so a rough
  // seed is walked in over several attempts instead of being retried from cold.
  std::optional<Transform> provisional_;
  int consecutive_failures_ = 0;

  mutable std::mutex stats_mutex_;
  Stats stats_;

  std::thread worker_;
};

}  // namespace bievr

#endif  // BIEVR_LOCALIZATION_LOCALIZER_H_
