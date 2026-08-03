#ifndef BIEVR_LIO_UTILS_H_
#define BIEVR_LIO_UTILS_H_

#include "bievr_lio/common.h"

namespace bievr {

inline uint64_t sToNs(const double s) { return static_cast<uint64_t>(s * 1e9); }

inline double nsToS(const uint64_t nsec) { return static_cast<double>(nsec) / 1e9; }

inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew_sym;
  skew_sym << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;
  return skew_sym;
}

inline Quaternion smallAnglePertubation(const V3& rot_vec) {
  return Quaternion(1.0, rot_vec.x() * 0.5, rot_vec.y() * 0.5, rot_vec.z() * 0.5);
}

inline Eigen::Matrix3d expMap(const Eigen::Vector3d& vec) {
  Eigen::AngleAxisd rot_axis(vec.norm(), vec.normalized());
  return rot_axis.toRotationMatrix();
}

inline Eigen::Vector3d logMap(const Eigen::Matrix3d& rot_mat) {
  Eigen::AngleAxisd rot_axis(rot_mat);
  return rot_axis.angle() * rot_axis.axis();
}

// Jacobian of (exp(theta) * M) w.r.t. theta evaluated at theta = 0,
// returned as a 9x3 stack of row-major rotation entries.
inline Eigen::Matrix<double, 9, 3> jacobianExpMapZeroM(const Eigen::Matrix3d& M) {
  Eigen::Matrix<double, 9, 3> output;
  output << 0, 0, 0, M(2, 0), M(2, 1), M(2, 2), -M(1, 0), -M(1, 1), -M(1, 2), -M(2, 0), -M(2, 1),
      -M(2, 2), 0, 0, 0, M(0, 0), M(0, 1), M(0, 2), M(1, 0), M(1, 1), M(1, 2), -M(0, 0), -M(0, 1),
      -M(0, 2), 0, 0, 0;
  return output;
}

inline M4 leftMultiply(const Quaternion& quat) {
  M4 left_mult;
  left_mult << quat.w(), -quat.x(), -quat.y(), -quat.z(), quat.x(), quat.w(), -quat.z(), quat.y(),
      quat.y(), quat.z(), quat.w(), -quat.x(), quat.z(), -quat.y(), quat.x(), quat.w();
  return left_mult;
}

inline M4 rightMultiply(const Quaternion& quat) {
  M4 right_mult;
  right_mult << quat.w(), -quat.x(), -quat.y(), -quat.z(), quat.x(), quat.w(), quat.z(), -quat.y(),
      quat.y(), -quat.z(), quat.w(), quat.x(), quat.z(), quat.y(), -quat.x(), quat.w();

  return right_mult;
}

// Generate a Gaussian kernel of given size and sigma
Eigen::MatrixXf buildGaussianKernel(int radius, float sigma);

float computeSigmaFromRadius(int radius);

std::vector<Point> getNeighborOffsets(double voxel_size);

// to save the pointcloud to a PCD file, return true if successful
bool savePointcloudPCD(const Pointcloud& cloud, const std::string& path);

// State carried between frames by the live status dashboard: the pre-indented
// ASCII art plus the accumulated trajectory length and the reference used to
// integrate it and the elapsed time.
struct DashboardState {
  std::string ascii;
  double trajectory_length = 0.0;
  V3 last_position = V3::Zero();
  bool has_last_position = false;
  double first_time_s = -1.0;
};

// Prints the dashboard ASCII art followed by a single boxed status line. Used
// to show a banner (e.g. "WAITING FOR DATA") before any frame is processed.
void printDashboardBanner(const std::string& ascii, const std::string& message);

// Prints the COIN-LIO style live status dashboard (position, orientation,
// velocity, trajectory length, biases, effective points, computation time).
// `stamp_ns` is the sensor time of the current frame; `velocity` is in the world
// frame; `comp_mean_s`/`comp_max_s` are the per-step computation time stats in
// seconds. `state` accumulates the trajectory length / elapsed-time reference.
void printDashboard(DashboardState& state, uint64_t stamp_ns, const Transform& T_W_I,
                    const V3& velocity, const V3& acc_bias, const V3& gyro_bias, double comp_mean_s,
                    double comp_max_s, int n_effective_points);

}  // namespace bievr

#endif  // BIEVR_LIO_UTILS_H_
