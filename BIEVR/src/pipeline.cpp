#include "bievr_lio/pipeline.h"

#include <fstream>
#include <sstream>

#include "bievr_lio/inertial_factor.h"
#include "bievr_lio/prior_factor.h"
#include "bievr_lio/timing.h"
#include "bievr_lio/undistort.h"
#include "bievr_lio/utils.h"

namespace bievr {

Pipeline::Pipeline(const Config& config) : config_(config) {
  map_ = std::make_shared<BIEVRMap>(config_.map);

  if (config_.accumulate_map) {
    map_accumulator_ = std::make_unique<MapAccumulator>(config_.map_save_resolution);
  }

  if (!config_.log_path.empty()) {
    LOG(I, "Logging to " << config_.log_path);
    tum_log_ = std::make_shared<std::ofstream>(config_.log_path, std::ios::trunc);
    if (!tum_log_->is_open()) {
      LOG(E, "Error opening output file.");
    }
  }

  if (config_.print_dashboard && !config_.dashboard_ascii_path.empty()) {
    std::ifstream ascii_file(config_.dashboard_ascii_path);
    if (ascii_file.is_open()) {
      // Indent every line so the art sits a few characters off the left margin.
      const std::string indent(6, ' ');
      std::string line;
      std::ostringstream art;
      while (std::getline(ascii_file, line)) {
        art << indent << line << "\n";
      }
      dashboard_.ascii = art.str();
    } else {
      LOG(W, "Could not open dashboard ASCII art at " << config_.dashboard_ascii_path);
    }
  }

  if (config_.print_dashboard) {
    printDashboardBanner(dashboard_.ascii, "BIEVR-LIO  WAITING FOR DATA");
  }
}

void Pipeline::processFrame(const std::vector<ImuMeasurement>& imu_data,
                            const StampedIntensityPointcloud& points_L) {
  timing::Timer step_timer("step");

  if (imu_data.empty()) {
    LOG(W, "No IMU data to process.");
    return;
  }

  // Cache the latest gyro reading so the odometry twist can report the current
  // angular velocity (already expressed in the body/IMU frame).
  latest_gyro_ = imu_data.back().gyro;

  // Remove points outside of intended range
  timing::Timer filter_timer("01_filter");
  StampedIntensityPointcloud points_filtered_L;
  filterMinMaxRange(points_L, points_filtered_L, config_.preprocess.min_range,
                    config_.preprocess.max_range);
  // The per-point time and intensity stay in points_filtered_L; we only take
  // zero-copy views onto those rows. Undistortion consumes the time view, intensity
  // rides through to publishing. Both views remain valid for the whole frame and
  // stay aligned by index with the spatial clouds derived below, since every
  // transform downstream preserves point order.
  const StampedIntensityPointcloud& filtered_L = points_filtered_L;
  const TimeView times = filtered_L.times();
  const IntensityView intensities = filtered_L.intensities();
  // Transform only the spatial coordinates into the IMU frame (single 3xN write);
  // time and intensity are never rewritten.
  const Pointcloud points_filtered_I = transformPoints(config_.T_I_L, points_filtered_L);
  filter_timer.Stop();

  if (phase_ == Phase::NeedBias) {
    // Estimate initial biases and orientation based on zero velocity assumption.
    // This also resolves imu_acc_scale_ (normalization detection).
    if (!initializeBias(imu_data, points_filtered_I)) return;
  }

  // Apply the accelerometer scale resolved during bias estimation if necessary.
  std::vector<ImuMeasurement> imu_scaled;
  const std::vector<ImuMeasurement>* imu_ptr = &imu_data;
  if (imu_acc_scale_ != 1.0) {
    imu_scaled = imu_data;
    for (auto& imu : imu_scaled) imu.acc *= imu_acc_scale_;
    imu_ptr = &imu_scaled;
  }
  const std::vector<ImuMeasurement>& imu = *imu_ptr;

  const State& x_i = states_.rbegin()->second;
  const Header header{points_filtered_L.end_stamp, static_cast<uint32_t>(x_i.id + 1),
                      config_.map_frame};

  // Propagate state based on IMU data
  timing::Timer preint_timer("02_preint");
  ImuIntegratorPtr imu_integrator =
      std::make_shared<ImuIntegrator>(config_.imu, acc_bias_, gyro_bias_);
  imu_integrator->integrate(imu);
  V3 G = gravity_dir_ * kGMagnitude;
  State x_j_pred;
  imu_integrator->predict(x_i.quat, x_i.p, x_i.v, G, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  preint_timer.Stop();

  if (points_filtered_I.empty()) {
    LOG(W, "No pointcloud data to process.");
    addState(imu.back().stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
    publishLatestState(header);
    return;
  }

  // Undistort pointcloud based on IMU integration
  timing::Timer undistortion_timer("03_undistortion");
  Pointcloud points_undistorted_I;
  undistortCloud(imu_integrator, x_i, points_filtered_I, times, points_filtered_L.stamp, G,
                 points_undistorted_I);
  undistortion_timer.Stop();

  std::vector<double> ranges;
  calculateRanges(points_undistorted_I, ranges);
  const Transform T_W_I_init(x_j_pred.quat, x_j_pred.p);
  if (phase_ == Phase::NeedMap) {
    tryInitMap(imu_data.back().stamp, x_j_pred, T_W_I_init, points_undistorted_I, intensities,
               ranges, header);
    return;
  }

  // Select points from the source cloud that will be used for registration
  timing::Timer voxel_timer("04_sampling");
  Pointcloud source_filtered, source_coarse, source_fine;
  sampleSource(points_undistorted_I, T_W_I_init, source_filtered, source_coarse, source_fine);
  voxel_timer.Stop();

  // Perform the actual registration
  timing::Timer align_timer("05_registration");
  LsqRegistration optimizer(*map_, source_filtered, config_.registration);
  const Transform T_W_I = optimizer.computeTransformation(T_W_I_init);
  const int n_effective_points = optimizer.numEffectivePoints();
  align_timer.Stop();

  // Transform the full cloud using the estimated pose and add it to the map
  timing::Timer map_timer("06_map");
  const Pointcloud points_registered = T_W_I * points_undistorted_I;
  map_->integratePoints(points_registered, &ranges);
  map_timer.Stop();

  // Bookkeeping and optimization of the intertial part of the state
  addState(imu.back().stamp, T_W_I.quaternion(), T_W_I.translation(), x_j_pred.v);

  timing::Timer imu_opt("07_imu_opt");
  if (map_->size() > config_.min_map_size_for_imu_opt) {
    addImuIntegrator(imu_integrator);
  }
  optimizeInertialWindow();
  imu_opt.Stop();

  // Publish results
  timing::Timer pub_time("08_publish");
  publishFrame(header, T_W_I, points_registered, source_filtered, source_coarse, source_fine,
               points_undistorted_I, intensities);
  pub_time.Stop();

  step_timer.Stop();

  if (!config_.log_path.empty()) {
    logTUM(nsToS(points_L.end_stamp), T_W_I);
  }

  if (config_.print_timing) {
    LOG(I, "Timings:\n" << timing::Timing::Print());
  }

  if (config_.print_dashboard) {
    printDashboard(dashboard_, header.stamp, T_W_I, x_j_pred.v, acc_bias_, gyro_bias_,
                   timing::Timing::GetMeanSeconds("step"), timing::Timing::GetMaxSeconds("step"),
                   n_effective_points);
  }
}

bool Pipeline::initializeBias(const std::vector<ImuMeasurement>& imu_data,
                              const Pointcloud& pointcloud) {
  if (!bias_initializer_) {
    bias_initializer_ =
        std::make_unique<BiasInitializer>(config_.imu.t_init, config_.imu.normalized);
  }
  for (const auto& imu : imu_data) {
    if (bias_initializer_->addMeasurement(imu)) break;
  }
  if (!bias_initializer_->isReady()) return false;

  acc_bias_ = bias_initializer_->accBias();
  gyro_bias_ = bias_initializer_->gyroBias();
  imu_acc_scale_ = bias_initializer_->accScale();
  const Rotation R_est = bias_initializer_->initialOrientation();
  bias_initializer_.reset();

  LOG(I, "Bias initialized: " << acc_bias_.transpose() << " " << gyro_bias_.transpose());
  const Eigen::Vector3d euler = R_est.eulerAngles(2, 1, 0);  // yaw, pitch, roll
  LOG(I, "Initial Pitch: " << (180. / M_PI) * euler[1]);
  LOG(I, "Initial Roll: " << (180. / M_PI) * euler[2]);

  State x_init;
  x_init.quat = R_est;
  x_init.p = V3::Zero();
  x_init.v = V3::Zero();

  addState(imu_data.back().stamp, x_init.quat, x_init.p, x_init.v);

  if (pointcloud.empty()) {
    phase_ = Phase::NeedMap;
  } else {
    const Transform T_W_I_init(x_init.quat, x_init.p);
    std::vector<double> ranges(pointcloud.size(), 0.f);
    for (size_t i = 0; i < pointcloud.size(); ++i) {
      ranges[i] = pointcloud[i].head<3>().norm();
    }
    map_->integratePoints(T_W_I_init * pointcloud, &ranges);
    phase_ = Phase::Running;
  }
  return true;
}

void Pipeline::tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                          const Pointcloud& undistorted, const IntensityView& intensities,
                          std::vector<double>& ranges, const Header& header) {
  if (undistorted.size() < config_.min_points_for_map_init) return;
  const Pointcloud registered = T_W_I_init * undistorted;
  map_->integratePoints(registered, &ranges);
  addState(stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  publishLatestState(header);
  accumulateMap(registered);
  publish(IntensityPointcloud(registered, intensities), header, "points/registered");
  if (map_->size() > config_.map_size_running_threshold) {
    phase_ = Phase::Running;
  }
}

void Pipeline::sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                            Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const {
  Pointcloud source_down;
  voxelDownsample(undistorted, source_down, config_.preprocess.downsample_resolution);
  if (config_.preprocess.informed_sampling) {
    sampleInformed(*map_, T_W_I_init, source_down, coarse, fine,
                   config_.preprocess.downsample_resolution, config_.informed_sample_count);
    filtered = fine + coarse;
  } else {
    filtered = source_down;
  }
}

bool Pipeline::addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v) {
  if (states_.find(time) != states_.end()) {
    LOG(D, "Time " << time << " already exists in the Pipeline. Skipping.");
    return false;
  }

  states_[time] = {seq_counter_++, quat, p, v};

  const double allowed_oldest_time_s = nsToS(time) - config_.imu.window_length_s;
  if (allowed_oldest_time_s < 0) return true;

  const uint64_t allowed_oldest_time = sToNs(allowed_oldest_time_s);
  for (auto it = states_.begin(); it != states_.end();) {
    // Remove states that are outside of the window length, along with their corresponding IMU
    // integrators
    if (it->first >= allowed_oldest_time) break;
    imu_integrators_.erase(it->first);
    it = states_.erase(it);
  }
  return true;
}

bool Pipeline::addImuIntegrator(ImuIntegratorPtr imu_integrator) {
  uint64_t time = imu_integrator->getFirstTime();
  if (states_.find(time) == states_.end()) {
    LOG(D, "Time " << time << " does not exist in the Pipeline. Skipping.");
    return false;
  }

  imu_integrators_[time] = imu_integrator;
  return true;
}

bool Pipeline::optimizeInertialWindow() {
  const size_t n_states = states_.size();
  const size_t n_imu = imu_integrators_.size();
  if (n_states == 0 || n_imu == 0) {
    LOG(D, "No states or IMU integrators to optimize.");
    return false;
  }

  // Skip if there are not enough IMU integrators to prevent instable optimization
  if (n_imu < config_.min_imu_integrators_for_opt) return false;

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::LossFunction* loss_function = new ceres::TrivialLoss();

  bool first = true;
  for (auto integrator_it : imu_integrators_) {
    ImuIntegratorPtr& imu_integrator = integrator_it.second;
    const uint64_t t_i = imu_integrator->getFirstTime();
    const uint64_t t_j = imu_integrator->getLastTime();
    State& state_i = states_.at(t_i);
    State& state_j = states_.at(t_j);

    ceres::CostFunction* cost_function = new InertialFactor(imu_integrator);

    // Add preintegrated IMU cost factors between consecutive states, with shared bias and gravity
    // parameters
    problem.AddResidualBlock(cost_function, loss_function, state_i.quat.coeffs().data(),
                             state_i.p.data(), state_i.v.data(), state_j.quat.coeffs().data(),
                             state_j.p.data(), state_j.v.data(), acc_bias_.data(),
                             gyro_bias_.data(), gravity_dir_.data());
    // Keep poses fixed as we only want to optimize over velocities, biases and gravity direction
    problem.SetParameterBlockConstant(state_i.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_j.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_i.p.data());
    problem.SetParameterBlockConstant(state_j.p.data());

    if (first) {
      problem.SetParameterBlockConstant(state_i.v.data());
      first = false;
    }
  }

  // Add constant prior to add some rigidity to the optimization problem and prevent gravity from
  // drifting
  ceres::CostFunction* prior_cost_function =
      new PriorFactor(gravity_dir_, config_.gravity_prior_weight);
  problem.AddResidualBlock(prior_cost_function, loss_function, gravity_dir_.data());
  ceres::Manifold* gravity_manifold = new ceres::SphereManifold<3>();
  problem.SetManifold(gravity_dir_.data(), gravity_manifold);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  return true;
}

void Pipeline::publishFrame(const Header& header, const Transform& T_W_I,
                            const Pointcloud& full_registered, const Pointcloud& source_filtered,
                            const Pointcloud& source_coarse, const Pointcloud& source_fine,
                            const Pointcloud& undistorted, const IntensityView& intensities) {
  if (config_.publish_all_clouds) {
    publishDebugClouds(source_filtered, source_coarse, source_fine, intensities, T_W_I,
                       header);
  }
  accumulateMap(full_registered);
  publish(IntensityPointcloud(full_registered, intensities), header, "points/registered");
  Header body_header = header;
  body_header.frame = config_.body_frame;
  // The undistorted cloud keeps its original point order, so the snapshotted
  // intensity row still lines up with it.
  publish(IntensityPointcloud(undistorted, intensities), body_header, "points/undistorted");
  publishLatestState(header);
  if (frame_observer_) frame_observer_(header.stamp, T_W_I, undistorted);
}

void Pipeline::publishLatestState(const Header& header) {
  if (states_.empty()) {
    LOG(W, "No states to publish.");
    return;
  }

  const State& latest_state = states_.rbegin()->second;
  Odometry odom;
  odom.pose = Transform(latest_state.quat, latest_state.p);
  // The state velocity is expressed in the world frame; rotate it into the body
  // frame so it matches the odometry message's child frame. The angular velocity
  // comes straight from the latest gyro measurement (already in the body frame).
  odom.linear_velocity = latest_state.quat.conjugate() * latest_state.v;
  odom.angular_velocity = latest_gyro_;
  publish(odom, header, "odom", config_.body_frame);
  publish(acc_bias_, header, "bias/acc");
  publish(gyro_bias_, header, "bias/gyro");
  publishPath(odom.pose, header);
}

void Pipeline::accumulateMap(const Pointcloud& registered) {
  // Copies the cloud into the worker's queue and returns; the hashing itself
  // happens off the odometry thread.
  if (map_accumulator_) map_accumulator_->add(registered);
}

bool Pipeline::saveMap(const std::string& path, std::string* written_path,
                       size_t* num_points) const {
  if (!map_accumulator_) {
    LOG(E, "Cannot save map: accumulation is disabled (map_save.accumulate).");
    return false;
  }

  std::string out = path.empty() ? config_.map_save_path : path;
  if (out.empty()) out = "bievr_map.pcd";

  if (!map_accumulator_->save(out, num_points)) return false;
  if (written_path != nullptr) *written_path = out;
  return true;
}

void Pipeline::publishPath(const Transform& pose, const Header& header) {
  if (!config_.publish_path) return;

  path_.poses.push_back(StampedPose{header.stamp, pose});
  if (config_.max_path_poses > 0 && path_.poses.size() > config_.max_path_poses) {
    const size_t excess = path_.poses.size() - config_.max_path_poses;
    path_.poses.erase(path_.poses.begin(), path_.poses.begin() + excess);
  }
  publish(path_, header, "path");
}

void Pipeline::publishDebugClouds(const Pointcloud& source_filtered,
                                  const Pointcloud& source_coarse, const Pointcloud& source_fine,
                                  const IntensityView& intensities, const Transform& T_W_I,
                                  const Header& header) {
  Pointcloud source_registered = T_W_I * source_filtered;
  Pointcloud fine_registered = T_W_I * source_fine;
  Pointcloud coarse_registered = T_W_I * source_coarse;
  publish(fine_registered, header, "points/fine");
  publish(coarse_registered, header, "points/coarse");
  publish(source_registered, header, "points/effective");
  // Header body_header = header;
  // body_header.frame = config_.body_frame;
  // The undistorted cloud keeps its original point order, so the snapshotted
  // intensity row still lines up with it.
  // publish(IntensityPointcloud(undistorted_cloud, intensities), body_header, "points/undistorted");
}

void Pipeline::logTUM(double timestamp, const Transform& pose) {
  const Quaternion q(pose.linear());
  const V3& t = pose.translation();
  if (!tum_log_->is_open()) {
    LOG(E, "Error: Output file is not open for logging.");
    return;
  }

  (*tum_log_) << std::fixed;
  (*tum_log_) << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << "\n";
}

}  // namespace bievr
