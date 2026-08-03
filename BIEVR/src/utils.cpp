#include "bievr_lio/utils.h"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>

#include "bievr_lio/log++.h"

namespace bievr {

bool savePointcloudPCD(const Pointcloud& cloud, const std::string& path) {
  namespace fs = std::filesystem;
  const fs::path out(path);
  if (out.has_parent_path()) {
    std::error_code ec;
    fs::create_directories(out.parent_path(), ec);
    if (ec) {
      LOG(E, "Cannot create directory '" << out.parent_path().string() << "': " << ec.message());
      return false;
    }
  }

  std::ofstream file(path, std::ios::binary);
  if (!file) {
    LOG(E, "Cannot open '" << path << "' for writing.");
    return false;
  }

  const size_t n = cloud.size();
  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z\n"
       << "SIZE 4 4 4\n"
       << "TYPE F F F\n"
       << "COUNT 1 1 1\n"
       << "WIDTH " << n << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << n << "\n"
       << "DATA binary\n";

  // The cloud stores doubles; PCD consumers expect float32, so convert in a
  // buffer rather than writing the Eigen storage straight out.
  std::vector<float> buffer(3 * n);
  for (size_t i = 0; i < n; ++i) {
    buffer[3 * i + 0] = static_cast<float>(cloud[i].x());
    buffer[3 * i + 1] = static_cast<float>(cloud[i].y());
    buffer[3 * i + 2] = static_cast<float>(cloud[i].z());
  }
  file.write(reinterpret_cast<const char*>(buffer.data()),
             static_cast<std::streamsize>(buffer.size() * sizeof(float)));
  if (!file) {
    LOG(E, "Failed while writing '" << path << "'.");
    return false;
  }
  return true;
}

// Width of the dashboard table (characters between the border bars).
constexpr int kDashWidth = 55;

// Horizontal border of the dashboard table.
std::string dashRule() { return "|" + std::string(kDashWidth, '-') + "|"; }

// A content row left-aligned and padded to the fixed table width.
std::string dashRow(const std::string& text) {
  std::ostringstream os;
  os << "| " << std::left << std::setw(kDashWidth - 1) << text << "|";
  return os.str();
}

// Fixed-width string for a double, matching the COIN-LIO dashboard formatting.
std::string dashFmt(double value) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << value;
  return os.str();
}
// Seconds rendered as milliseconds with a single decimal.
std::string dashMs(double seconds) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << seconds * 1e3;
  return os.str();
}

void printDashboardBanner(const std::string& ascii, const std::string& message) {
  std::ostringstream out;
  out << "\033[2J\033[1;1H";  // clear screen and move cursor home
  if (!ascii.empty()) {
    out << ascii << "\n";
  }
  out << dashRule() << "\n";
  out << dashRow(message) << "\n";
  out << dashRule();
  std::cout << out.str() << std::endl;
}

void printDashboard(DashboardState& state, uint64_t stamp_ns, const Transform& T_W_I,
                    const V3& velocity, const V3& acc_bias, const V3& gyro_bias, double comp_mean_s,
                    double comp_max_s, int n_effective_points) {
  // Accumulate the travelled distance from the previous reported position.
  const V3 position = T_W_I.translation();
  if (state.has_last_position) {
    state.trajectory_length += (position - state.last_position).norm();
  }
  state.last_position = position;
  state.has_last_position = true;

  const std::string rule = dashRule();
  const auto& row = dashRow;

  const Eigen::Vector3d euler = T_W_I.rotation().eulerAngles(0, 1, 2) * (180.0 / M_PI);

  // Header line: sensor time and elapsed time since the first dashboard frame.
  const double current_time_s = nsToS(stamp_ns);
  if (state.first_time_s < 0.0) state.first_time_s = current_time_s;
  const double elapsed_s = current_time_s - state.first_time_s;
  const std::time_t current_time = static_cast<std::time_t>(current_time_s);
  char time_buf[32] = "";
  std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&current_time));

  std::ostringstream out;
  out << "\033[2J\033[1;1H";  // clear screen and move cursor home
  if (!state.ascii.empty()) {
    out << state.ascii << "\n";
  }
  out << rule << "\n";
  out << row("BIEVR-LIO  " + std::string(time_buf) + "  Elapsed: " + dashFmt(elapsed_s) + " s")
      << "\n";
  out << rule << "\n";
  out << row("Position (x,y,z)    [m]   : " + dashFmt(position.x()) + " " + dashFmt(position.y()) +
             " " + dashFmt(position.z()))
      << "\n";
  out << row("Orientation (r,p,y) [deg] : " + dashFmt(euler.x()) + " " + dashFmt(euler.y()) + " " +
             dashFmt(euler.z()))
      << "\n";
  out << row("Velocity (x,y,z)    [m/s] : " + dashFmt(velocity.x()) + " " + dashFmt(velocity.y()) +
             " " + dashFmt(velocity.z()))
      << "\n";
  out << row("Trajectory Length   [m]   : " + dashFmt(state.trajectory_length)) << "\n";
  out << rule << "\n";
  out << row("Acc. Bias (x,y,z) [m/s2]  : " + dashFmt(acc_bias.x()) + " " + dashFmt(acc_bias.y()) +
             " " + dashFmt(acc_bias.z()))
      << "\n";
  out << row("Gyro Bias (x,y,z) [rad/s] : " + dashFmt(gyro_bias.x()) + " " +
             dashFmt(gyro_bias.y()) + " " + dashFmt(gyro_bias.z()))
      << "\n";
  out << rule << "\n";
  out << row("Effective Points      [#] : " + std::to_string(n_effective_points)) << "\n";
  out << row("Computation Time     [ms] : Avg: " + dashMs(comp_mean_s) +
             " Max: " + dashMs(comp_max_s))
      << "\n";
  out << rule;

  std::cout << out.str() << std::endl;
}

Eigen::MatrixXf buildGaussianKernel(int radius, float sigma) {
  int size = 2 * radius + 1;
  Eigen::MatrixXf kernel(size, size);
  float sum = 0.0f;

  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      float value = std::exp(-(x * x + y * y) / (2 * sigma * sigma));
      kernel(y + radius, x + radius) = value;
      sum += value;
    }
  }
  kernel /= sum;  // normalize
  return kernel;
}

float computeSigmaFromRadius(int radius) {
  int ksize = 2 * radius + 1;
  return 0.3f * ((ksize - 1) * 0.5f - 1) + 0.8f;
}

// Offsets to the 7-connected neighbors of a voxel: the voxel itself plus ±voxel_size along each
// axis. Order is {0}, +x, -x, +y, -y, +z, -z.
std::vector<Point> getNeighborOffsets(double voxel_size) {
  std::vector<Point> offsets;
  offsets.reserve(7);
  offsets.push_back(Point::Zero());
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {1.0, -1.0}) {
      Point offset = Point::Zero();
      offset(axis) = sign * voxel_size;
      offsets.push_back(offset);
    }
  }
  return offsets;
}

}  // namespace bievr
