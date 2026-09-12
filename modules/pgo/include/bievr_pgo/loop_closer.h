#ifndef BIEVR_PGO_LOOP_CLOSER_H_
#define BIEVR_PGO_LOOP_CLOSER_H_

// Scan Context loop closure + GTSAM pose graph, running downstream of the
// odometry. 
// Feed it with Pipeline::setFrameObserver. addFrame() only decides whether the
// frame is a keyframe and hands a copy to a worker, so the odometry thread never
// waits on ICP or iSAM2.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "bievr_lio/common.h"
#include "bievr_map_io/prior_map.h"
#include "bievr_scancontext/scan_context.h"

namespace bievr {

class LoopCloser {
 public:
  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;

  struct Config {
    // Keyframing: a frame is kept once either threshold is exceeded.
    double keyframe_meter_gap = 1.0;
    double keyframe_deg_gap = 15.0;

    double keyframe_filter_size = 0.4;  // voxel leaf of the stored keyframe cloud [m]
    double icp_filter_size = 0.4;       // voxel leaf of the loop-closure submap [m]

    ScanContext::Config scan_context;
    double loop_detect_frequency = 1.0;  // Hz

    int history_keyframe_search_num = 25;  // [-N, N] keyframes stacked into the target submap
    double icp_max_correspondence_distance = 150.0;
    int icp_max_iterations = 100;
    double icp_transformation_epsilon = 1e-6;
    double icp_euclidean_fitness_epsilon = 1e-6;
    int icp_ransac_iterations = 0;
    double loop_fitness_score_threshold = 0.3;  // reject loops scoring above this
    // Seed ICP with Scan Context's yaw instead of identity.
    // only useful where revisits run in the opposite direction.
    bool use_sc_yaw_guess = false;
    double sc_yaw_guess_min_deg = 45.0;

    double prior_noise_score = 1e-12;
    double odom_noise_rotation = 1e-6;
    double odom_noise_translation = 1e-4;
    double loop_noise_score = 0.5;
    double loop_noise_cauchy_c = 1.0;

    // GPS config
    bool use_gps_altitude = false;
    double gps_noise_z = 3.0;         // sigma of the altitude measurement [m]
    double gps_max_time_diff = 0.15;  // fix must be this close to the keyframe [s]
    double gps_min_distance = 5.0;    // travel between two GPS factors [m]
    double gps_max_variance = 25.0;   // reject fixes reporting worse than this [m^2]
    size_t gps_max_buffer = 400;

    double isam_relinearize_threshold = 0.01;
    int isam_relinearize_skip = 1;
    double isam_frequency = 10.0;  // Hz

    size_t max_queue = 8;  // pending keyframes before the oldest are dropped

    double map_save_resolution = 0.1;  // voxel leaf of the saved cloud [m]; <=0 keeps full res
    std::string map_frame = "odom";    // recorded in the bundle's meta.yaml
  };

  struct Keyframe {
    uint64_t stamp = 0;
    Transform pose;  // optimized
  };

  struct Stats {
    size_t num_keyframes = 0;
    size_t num_loops = 0;      // loop factors accepted
    size_t num_rejected = 0;   // candidates that failed the fitness test
    size_t dropped_frames = 0;
    size_t num_gps = 0;  // GPS factors added

  };

  explicit LoopCloser(Config config);
  ~LoopCloser();

  LoopCloser(const LoopCloser&) = delete;
  LoopCloser& operator=(const LoopCloser&) = delete;

  // Odometry-thread entry point. Returns immediately.
  void addFrame(uint64_t stamp, const Transform& T_W_I, const Pointcloud& cloud_body);
  void addGps(uint64_t stamp, double altitude, double variance);

  std::vector<Keyframe> keyframes() const;
  Stats stats() const;

  // Nothing queued and no unoptimized factors. Bag replay waits on this before
  // saving, since the workers lag the feed.
  bool idle() const;

  // World-frame cloud of every keyframe at its optimized pose. `resolution` <= 0
  // keeps full resolution.
  Pointcloud buildMap(double resolution) const;

  // Writes cloud.pcd, poses_tum.txt, scan_context.bin and meta.yaml into `dir`.
  bool saveMapBundle(const std::string& dir, std::string* message = nullptr) const;

 private:
  struct PendingFrame {
    uint64_t stamp;
    Transform pose;
    Cloud::Ptr cloud;  // body frame, downsampled
  };

  struct GpsSample {
    uint64_t stamp;
    double altitude;
    double variance;
  };

  struct LoopCandidate {
    int from;  // older keyframe
    int to;    // newer keyframe
    float yaw_diff_rad;
  };

  // One consistent view of the keyframe store, taken under a single lock so the
  // saved cloud and poses cannot come from different iSAM2 snapshots.
  struct Snapshot {
    std::vector<Cloud::Ptr> clouds;
    std::vector<gtsam::Pose3> poses;
    std::vector<uint64_t> stamps;
  };
  Snapshot snapshot() const;

  void keyframeWorker();
  void loopDetectWorker();
  void icpWorker();
  void isamWorker();

  // Sleeps for `period`, waking early on shutdown. Returns true if stopping.
  bool waitOrStop(std::chrono::duration<double> period);

  void integrateKeyframe(PendingFrame frame);
  // Nearest fix to `stamp`, or nullopt when none is close enough.
  std::optional<GpsSample> gpsAt(uint64_t stamp) const;
  // add altitude factor for keyframe 'index'
  void maybeAddGpsFactor(int index, const gtsam::Pose3& pose, uint64_t stamp);
  // World-frame submap of keyframes within +-span of `key`, each at its own pose.
  Cloud::Ptr buildSubmap(int key, int span) const;
  // Every keyframe at its optimized pose, voxel-thinned when resolution > 0.
  Cloud::Ptr accumulate(const Snapshot& snap, double resolution) const;
  // Relative constraint from ICP, or nullopt if the fitness test rejects it.
  std::optional<gtsam::Pose3> computeLoopConstraint(const LoopCandidate& candidate);
  void optimize();
  void initNoiseModels();

  Config config_;
  double keyframe_rad_gap_;

  // Keyframe gating state, odometry thread only.
  bool has_previous_frame_ = false;
  Transform previous_pose_;
  double translation_accumulated_ = 0.0;
  double rotation_accumulated_ = 0.0;

  // Shutdown signal. Each waiter re-checks it under its own mutex, so the
  // destructor must touch every one of them before notifying.
  std::atomic<bool> stop_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  // Only keyframeWorker waits on queue_cv_, so addFrame may notify_one.
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PendingFrame> queue_;

  mutable std::mutex keyframe_mutex_;
  std::vector<Cloud::Ptr> keyframe_clouds_;
  std::vector<gtsam::Pose3> keyframe_odom_poses_;
  std::vector<gtsam::Pose3> keyframe_poses_;  // optimized
  std::vector<uint64_t> keyframe_stamps_;

  mutable std::mutex sc_mutex_;
  ScanContext scan_context_;

  mutable std::mutex candidate_mutex_;
  std::condition_variable candidate_cv_;
  std::deque<LoopCandidate> candidates_;

  mutable std::mutex graph_mutex_;

  // gps altitude mutex and buffer 
  mutable std::mutex gps_mutex_;
  std::deque<GpsSample> gps_buffer_; 
  // anchor gps ( z = 0)
  std::optional<double> gps_anchor_altitude_;
  std::optional<gtsam::Point3> last_gps_position_;
  std::atomic<size_t> num_gps_{0};

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  bool graph_initialized_ = false;

  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel loop_noise_;
  gtsam::SharedNoiseModel gps_noise_;

  std::atomic<size_t> num_loops_{0};
  std::atomic<size_t> num_rejected_{0};
  std::atomic<size_t> dropped_frames_{0};

  std::vector<std::thread> workers_;
};

}  // namespace bievr

#endif  // BIEVR_PGO_LOOP_CLOSER_H_
