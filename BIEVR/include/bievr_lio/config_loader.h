#ifndef BIEVR_LIO_CONFIG_LOADER_H_
#define BIEVR_LIO_CONFIG_LOADER_H_

// Single source of truth for the wrapper configuration. This header is
// ROS-agnostic: it reads plain YAML files via yaml-cpp and fills `Config`, so
// the ROS1 and ROS2 wrappers share one loader and one set of YAML config files.
// The ROS layer only carries the *paths* of the files to load.
//
// The YAML is organised into sections (topics / calibration / lidar / imu /
// map / preprocess / optimization / debug). Files are
// layered: callers pass {params, sensor_config} and the later file wins on a
// per-leaf basis, so the sensor config wins on any leaf both files define and a
// section that appears in both (e.g. `imu`) merges by key.
//
// Consumers need yaml-cpp on their include/link path (header-only here; the core
// library itself does not compile or depend on it).

#include <bievr_lio/bievr_map.h>
#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/pipeline.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <ostream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace bievr {

struct TopicConfig {
  std::string pointcloud_topic = "/points";
  std::string imu_topic = "/imu";
  std::string bag_path = "";
};

struct Config {
  TopicConfig topic_config;
  Pipeline::Config pipeline_config;
  // Upper bound on threads used for TBB parallel regions. 0 = let TBB decide
  // (use all available cores); >0 caps parallelism to that many threads.
  int max_num_threads = 0;
};

namespace config_internal {

// A read-only view over one or more YAML documents. Lookups address a
// `section.key` pair and scan the documents in reverse, so the last file that
// defines that exact leaf wins (sections shared across files merge per key).
class MergedYaml {
 public:
  void add(const YAML::Node& node) { nodes_.push_back(node); }

  template <typename T>
  T get(const std::string& section, const std::string& key, const T& default_value) const {
    for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
      const YAML::Node& root = *it;
      if (root[section] && root[section][key]) {
        return root[section][key].as<T>();
      }
    }
    return default_value;
  }

  // Same as get(), but for a key that lives at the document root (no section).
  template <typename T>
  T getTopLevel(const std::string& key, const T& default_value) const {
    for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
      const YAML::Node& root = *it;
      if (root[key]) {
        return root[key].as<T>();
      }
    }
    return default_value;
  }

 private:
  std::vector<YAML::Node> nodes_;
};

// Builds a Transform from a section holding `translation` (3) and `rotation` (9)
// vectors. Extrinsics are mandatory: a missing section yields empty vectors and
// an incomplete section yields wrong-sized ones, both of which are hard errors
// (no silent fallback to identity). Returns false and leaves `out` untouched on
// failure.
inline bool extrinsicFromVectors(const std::vector<double>& t_vec, const std::vector<double>& R_vec,
                                 const std::string& label, Transform& out) {
  if (t_vec.size() != 3) {
    LOG(E, "Config error: " << label << " translation must have 3 elements, got " << t_vec.size()
                            << ". Extrinsics must be provided completely.");
    return false;
  }
  if (R_vec.size() != 9) {
    LOG(E, "Config error: " << label << " rotation must have 9 elements, got " << R_vec.size()
                            << ". Extrinsics must be provided completely.");
    return false;
  }
  const V3 t(t_vec[0], t_vec[1], t_vec[2]);
  Rotation R;
  R << R_vec[0], R_vec[1], R_vec[2], R_vec[3], R_vec[4], R_vec[5], R_vec[6], R_vec[7], R_vec[8];
  out = Transform(R, t);
  return true;
}

// Reads a scalar that is allowed to be absent (the positive `default_value` is
// used then) but, when present, must be positive. A non-positive value is a hard
// error: reports it and returns false. On success writes the value to `out`.
template <typename T>
bool getPositive(const MergedYaml& yaml, const std::string& section, const std::string& key,
                 const T& default_value, T& out) {
  out = yaml.get<T>(section, key, default_value);
  if (out <= T(0)) {
    LOG(E, "Config error: '" << section << "." << key << "' must be positive, got " << out << ".");
    return false;
  }
  return true;
}

// Appends a transform as a translation vector and a flat row-major rotation, so
// the overview stays on a couple of lines per extrinsic.
inline void printExtrinsic(std::ostream& os, const std::string& label, const Transform& T) {
  const auto& t = T.translation();
  const auto& R = T.rotation();
  os << "  " << label << " t:  [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
  os << "  " << label << " R:  [" << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", "
     << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", " << R(2, 0) << ", " << R(2, 1) << ", "
     << R(2, 2) << "]\n";
}

// Dumps every parameter the loader resolved (defaults included), so the run log
// shows exactly what configuration was used.
inline void printConfigOverview(const Config& config) {
  const auto& tc = config.topic_config;
  const auto& hc = config.pipeline_config;
  auto yn = [](bool b) { return b ? "true" : "false"; };

  std::ostringstream os;
  os << "================ BIEVR-LIO config ================\n";
  os << "topics:\n";
  os << "  pointcloud:           " << tc.pointcloud_topic << "\n";
  os << "  imu:                  " << tc.imu_topic << "\n";
  os << "max_num_threads:        "
     << (config.max_num_threads > 0 ? std::to_string(config.max_num_threads)
                                    : std::string("automatic"))
     << "\n";
  os << "lidar:\n";
  os << "  min_range_m:          " << hc.preprocess.min_range << "\n";
  os << "  max_range_m:          " << hc.preprocess.max_range << "\n";
  os << "map:\n";
  os << "  pixel_size_m:   " << hc.map.px_size << "\n";
  os << "  voxel_size_m:   " << hc.map.voxel_size << "\n";
  os << "  normal_tolerance_deg: " << hc.map.norm_tol_deg << "\n";
  os << "  max_size:             " << hc.map.max_size << "\n";
  os << "  smooth:               " << yn(hc.map.smooth) << "\n";
  os << "  weighted:             " << yn(hc.map.weighted) << "\n";
  os << "  frame:                " << hc.map_frame << "\n";
  os << "preprocess:\n";
  os << "  downsample_res_m:     " << hc.preprocess.downsample_resolution << "\n";
  os << "  informed_sampling:    " << yn(hc.preprocess.informed_sampling) << "\n";
  os << "optimization:\n";
  os << "  huber_delta:          " << hc.registration.huber_delta << "\n";
  os << "  img_residual:         " << yn(hc.registration.img_residual) << "\n";
  os << "  img_jacobian:         " << yn(hc.registration.img_jacobian) << "\n";
  os << "imu:\n";
  os << "  window_s:             " << hc.imu.window_length_s << "\n";
  os << "  t_init:               " << hc.imu.t_init << "\n";
  os << "  normalized:           " << hc.imu.normalized << "\n";
  os << "  frame:                " << hc.body_frame << "\n";
  os << "path:\n";
  os << "  publish:              " << yn(hc.publish_path) << "\n";
  os << "  max_poses:            "
     << (hc.max_path_poses > 0 ? std::to_string(hc.max_path_poses) : std::string("unlimited"))
     << "\n";
  os << "map_save:\n";
  os << "  accumulate:           " << yn(hc.accumulate_map) << "\n";
  os << "  resolution_m:         " << hc.map_save_resolution << "\n";
  os << "  path:                 "
     << (hc.map_save_path.empty() ? "<cwd>/bievr_map.pcd" : hc.map_save_path) << "\n";
  os << "debug:\n";
  os << "  publish_all_clouds:        " << yn(hc.publish_all_clouds) << "\n";
  os << "  print_timing:         " << yn(hc.print_timing) << "\n";
  os << "  print_debug:          " << yn(hc.print_debug) << "\n";
  os << "  dashboard:            " << yn(hc.print_dashboard) << "\n";
  if (hc.print_dashboard) {
    os << "  dashboard_ascii_path: "
       << (hc.dashboard_ascii_path.empty() ? "<none>" : hc.dashboard_ascii_path) << "\n";
  }
  os << "  log_path:             " << (hc.log_path.empty() ? "<none>" : hc.log_path) << "\n";
  os << "calibration (LiDAR -> IMU):\n";
  printExtrinsic(os, "T_I_L", hc.T_I_L);
  os << "==================================================";
  LOG(I, os.str());
}

}  // namespace config_internal

// Loads `Config` from one or more plain YAML files (no ROS parameter server).
// Later files override earlier ones per leaf key. Returns false if a file cannot
// be opened or parsed.
inline bool loadConfigFromYaml(const std::vector<std::string>& yaml_paths, Config& config) {
  config_internal::MergedYaml yaml;
  for (const std::string& path : yaml_paths) {
    if (path.empty()) {
      continue;
    }
    LOG(I, "Loading config from '" << path << "'.");
    try {
      yaml.add(YAML::LoadFile(path));
    } catch (const std::exception& e) {
      LOG(E, "Failed to load YAML config '" << path << "': " << e.what());
      return false;
    }
  }

  auto& tc = config.topic_config;
  auto& hc = config.pipeline_config;

  // --- topics ---
  tc.pointcloud_topic = yaml.get<std::string>("topics", "pointcloud", tc.pointcloud_topic);
  tc.imu_topic = yaml.get<std::string>("topics", "imu", tc.imu_topic);

  // --- lidar ---
  hc.preprocess.min_range = yaml.get<double>("lidar", "min_range_m", 0.5);
  hc.preprocess.max_range = yaml.get<double>("lidar", "max_range_m", 100.);

  // --- map ---
  // These resolutions/tolerances are optional (fall back to the defaults below)
  // but must be positive when supplied; a non-positive value aborts the load.
  int max_size = 0;
  if (!config_internal::getPositive(yaml, "map", "pixel_size_m", 0.05, hc.map.px_size) ||
      !config_internal::getPositive(yaml, "map", "voxel_size_m", 0.5, hc.map.voxel_size) ||
      !config_internal::getPositive(yaml, "map", "normal_tolerance_deg", 3., hc.map.norm_tol_deg) ||
      !config_internal::getPositive(yaml, "map", "max_size", 5000000, max_size)) {
    return false;
  }
  hc.map.max_size = static_cast<size_t>(max_size);
  hc.map.smooth = yaml.get<bool>("map", "smooth", false);
  hc.map.weighted = yaml.get<bool>("map", "weighted", false);
  // The map frame is the parent (odometry) frame for published poses/clouds.
  hc.map_frame = yaml.get<std::string>("map", "frame", hc.map_frame);

  // --- preprocess ---
  if (!config_internal::getPositive(yaml, "preprocess", "downsample_resolution_m", 0.15,
                                    hc.preprocess.downsample_resolution)) {
    return false;
  }
  hc.preprocess.informed_sampling = yaml.get<bool>("preprocess", "informed_sampling", false);

  // --- optimization ---
  if (!config_internal::getPositive(yaml, "optimization", "huber_delta", 100.,
                                    hc.registration.huber_delta)) {
    return false;
  }
  hc.registration.img_residual = yaml.get<bool>("optimization", "img_residual", true);
  hc.registration.img_jacobian = yaml.get<bool>("optimization", "img_jacobian", true);

  // --- imu (params side: inertial window + normalization) ---
  if (!config_internal::getPositive(yaml, "imu", "window_s", 10., hc.imu.window_length_s) ||
      !config_internal::getPositive(yaml, "imu", "t_init", 0.2, hc.imu.t_init)) {
    return false;
  }
  // Accelerometer normalization, resolved during bias estimation:
  //   < 0 autodetect, = 0 not normalized, > 0 normalized (multiplied by g)
  hc.imu.normalized = yaml.get<double>("imu", "normalized", -1.0);
  // The IMU frame is the body (child) frame of the published odometry.
  hc.body_frame = yaml.get<std::string>("imu", "frame", hc.body_frame);

  // --- path (published trajectory) ---
  hc.publish_path = yaml.get<bool>("path", "publish", true);
  // Unlike the resolutions above, 0 is meaningful here (unlimited), so this is
  // checked for negativity rather than run through getPositive().
  const int max_path_poses = yaml.get<int>("path", "max_poses", 10000);
  if (max_path_poses < 0) {
    LOG(E, "Config error: 'path.max_poses' must be >= 0 (0 = unlimited), got " << max_path_poses
                                                                               << ".");
    return false;
  }
  hc.max_path_poses = static_cast<size_t>(max_path_poses);

  // --- map saving (accumulated registered scans, written by the save service) ---
  hc.accumulate_map = yaml.get<bool>("map_save", "accumulate", true);
  hc.map_save_path = yaml.get<std::string>("map_save", "path", "");
  if (!config_internal::getPositive(yaml, "map_save", "resolution_m", 0.1,
                                    hc.map_save_resolution)) {
    return false;
  }

  // --- debug ---
  hc.publish_all_clouds = yaml.get<bool>("debug", "publish_all_clouds", false);
  hc.print_timing = yaml.get<bool>("debug", "timing", false);
  hc.print_debug = yaml.get<bool>("debug", "log", false);
  hc.log_path = yaml.get<std::string>("debug", "trajectory_path", "");

  // --- dashboard (live status print) ---
  hc.print_dashboard = yaml.get<bool>("debug", "dashboard", false);
  hc.dashboard_ascii_path = yaml.get<std::string>("debug", "dashboard_ascii_path", "");
  // If the dashboard is enabled but no explicit art path is given, look for
  // `bievr_ascii.txt` next to the config files that were loaded. Those files may
  // live in different directories (e.g. params.yaml in config/, the sensor
  // config in config/sensor_configs/), so probe each config's directory and its
  // parent and take the first that actually contains the file rather than
  // guessing a path that may not exist.
  if (hc.print_dashboard && hc.dashboard_ascii_path.empty()) {
    namespace fs = std::filesystem;
    for (const std::string& p : yaml_paths) {
      if (p.empty()) continue;
      const fs::path dir = fs::path(p).parent_path();
      for (const fs::path& candidate_dir : {dir, dir.parent_path()}) {
        const fs::path candidate = candidate_dir / "bievr_ascii.txt";
        std::error_code ec;
        if (fs::exists(candidate, ec)) {
          hc.dashboard_ascii_path = candidate.string();
          break;
        }
      }
      if (!hc.dashboard_ascii_path.empty()) break;
    }
  }

  // Lower the log level so DEBUG messages are shown when requested, otherwise
  // keep the default (INFO and above).
  LOG_SET_LEVEL(hc.print_debug ? BaseSeverity::DEBUG : BaseSeverity::INFO);

  // --- calibration (LiDAR -> IMU) ---
  if (!config_internal::extrinsicFromVectors(
          yaml.get<std::vector<double>>("calibration", "translation", {}),
          yaml.get<std::vector<double>>("calibration", "rotation", {}), "calibration", hc.T_I_L)) {
    return false;
  }

  // --- threading (top-level, process-wide) ---
  config.max_num_threads = yaml.getTopLevel<int>("max_num_threads", 0);
  if (config.max_num_threads < 0) {
    LOG(E, "Config error: 'max_num_threads' must be >= 0 (0 = automatic), got "
               << config.max_num_threads << ".");
    return false;
  }

  config_internal::printConfigOverview(config);
  return true;
}

// Loads `Config` from the executable's command-line arguments (the launch file
// passes the YAML paths as argv rather than ROS parameters, so this is fully
// ROS-agnostic and shared by both wrappers). Recognised flags:
//   --sensor_config_file <path>   --params_file <path>   --bag <path>
// `args` is the argument list with any ROS-specific arguments already stripped
// (ros::init does this in place on ROS1; rclcpp::remove_ros_arguments on ROS2).
inline bool loadConfigFromArgs(const std::vector<std::string>& args, Config& config) {
  std::string sensor_config_file;
  std::string params_file;
  std::string bag_path;
  bool have_bag = false;

  for (size_t i = 1; i < args.size(); ++i) {
    auto value = [&](const char* flag) -> std::string {
      if (i + 1 >= args.size()) {
        LOG(E, "Missing value after " << flag << ".");
        return "";
      }
      return args[++i];
    };
    if (args[i] == "--sensor_config_file") {
      sensor_config_file = value("--sensor_config_file");
    } else if (args[i] == "--params_file") {
      params_file = value("--params_file");
    } else if (args[i] == "--bag") {
      bag_path = value("--bag");
      have_bag = true;
    } else {
      LOG(W, "Ignoring unrecognized argument '" << args[i] << "'.");
    }
  }

  // Later files override earlier ones: params first, then sensor config, so the
  // sensor file wins on any leaf the two files share.
  if (!loadConfigFromYaml({params_file, sensor_config_file}, config)) {
    return false;
  }
  // The bag path is a launch argument, not part of the shared YAML config; only
  // override the config value when it was actually passed on the command line.
  if (have_bag) {
    config.topic_config.bag_path = bag_path;
  }
  return true;
}

}  // namespace bievr

#endif  // BIEVR_LIO_CONFIG_LOADER_H_
