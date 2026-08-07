#ifndef BIEVR_LIO_PIPELINE_H_
#define BIEVR_LIO_PIPELINE_H_

#include <typeindex>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/imu_integrator.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/map_accumulator.h"
#include "bievr_lio/preprocess.h"
#include "bievr_lio/utils.h"

namespace bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    BIEVRMap::Config map;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool publish_path = true;  
    size_t max_path_poses = 10000;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";
    // Accumulate the registered scans into a world-frame cloud that saveMap()
    // can write out. Off means saveMap() has nothing to save.
    bool accumulate_map = true;
    double map_save_resolution = 0.1;
    std::string map_save_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  const Config& config() const { return config_; }

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  // Writes the accumulated registered cloud as a binary PCD. An empty `path`
  // falls back to config.map_save_path (itself defaulting to ./bievr_map.pcd).
  // On success `written_path` reports where it landed and `num_points` how many
  // points were written. Fails if nothing has been accumulated yet.
  // Safe to call while the odometry runs: the map is snapshotted under a lock
  // and written outside it.
  bool saveMap(const std::string& path, std::string* written_path = nullptr,
               size_t* num_points = nullptr) const;

  // Number of points currently held in the accumulated map.
  size_t accumulatedMapSize() const { return map_accumulator_ ? map_accumulator_->size() : 0; }

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

  // Consumers of every registered frame, for downstream modules (loop closure,
  // localization). Called on the odometry thread, so they must copy and return
  // rather than do work inline. A list rather than a single slot so two modules
  // in one process cannot silently steal it from each other. None by default.
  using FrameObserver = std::function<void(uint64_t stamp, const Transform& T_W_I,
                                           const Pointcloud& undistorted_I)>;
  void addFrameObserver(FrameObserver observer) {
    frame_observers_.push_back(std::move(observer));
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data, const Pointcloud& pointcloud);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const IntensityView& intensities,
                  std::vector<double>& ranges, const Header& header);
  void sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                    Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const;

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const IntensityView& intensities);
  void publishLatestState(const Header& header);
  void publishPath(const Transform& pose, const Header& header);
  void accumulateMap(const Pointcloud& registered);
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header);

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::shared_ptr<BIEVRMap> map_;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Full odometry trajectory, republished as a path with every state publish.
  Path path_;
  // Collects the registered scans for saveMap() on its own thread. Null when
  // map accumulation is disabled.
  std::unique_ptr<MapAccumulator> map_accumulator_;
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::vector<FrameObserver> frame_observers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_PIPELINE_H_
