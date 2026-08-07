#include "bievr_map_io/prior_map.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {
namespace {

constexpr const char* kCloudFile = "cloud.pcd";
constexpr const char* kPosesFile = "poses_tum.txt";
constexpr const char* kDescriptorFile = "scan_context.bin";
constexpr const char* kMetaFile = "meta.yaml";

bool loadCloud(const std::string& path, double voxel_size, MapCloud::Ptr& out) {
  MapCloud::Ptr raw(new MapCloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *raw) != 0 || raw->empty()) return false;

  if (voxel_size <= 0.0) {
    out = raw;
    return true;
  }
  MapCloud::Ptr filtered(new MapCloud());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.setInputCloud(raw);
  filter.filter(*filtered);
  out = filtered;
  return true;
}

// A foreign PCD carries no conventions: wrong units or a non z-up axis show up
// here as an obviously wrong extent, long before ICP starts failing to converge.
void logExtent(const MapCloud& cloud) {
  Eigen::Vector3f lo = cloud.points.front().getVector3fMap();
  Eigen::Vector3f hi = lo;
  for (const auto& p : cloud.points) {
    lo = lo.cwiseMin(p.getVector3fMap());
    hi = hi.cwiseMax(p.getVector3fMap());
  }
  const Eigen::Vector3f extent = hi - lo;
  LOG(I, "Prior map: " << cloud.size() << " points, extent " << extent.x() << " x " << extent.y()
                       << " x " << extent.z() << " m.");
}

// TUM: `t tx ty tz qx qy qz qw`, one pose per line, seconds.
bool loadPoses(const std::string& path, std::vector<Transform>& poses,
               std::vector<uint64_t>& stamps) {
  std::ifstream file(path);
  if (!file) return false;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream stream(line);
    double t, tx, ty, tz, qx, qy, qz, qw;
    if (!(stream >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) return false;
    const Quaternion q(qw, qx, qy, qz);
    poses.emplace_back(q.normalized(), Point(tx, ty, tz));
    stamps.push_back(sToNs(t));
  }
  return !poses.empty();
}

void loadMeta(const std::string& path, MapMeta& meta) {
  const YAML::Node root = YAML::LoadFile(path);
  if (root["frame"]) meta.frame = root["frame"].as<std::string>();
  if (root["num_keyframes"]) meta.num_keyframes = root["num_keyframes"].as<size_t>();
  if (root["num_points"]) meta.num_points = root["num_points"].as<size_t>();
  if (root["map_save_resolution_m"]) {
    meta.map_save_resolution_m = root["map_save_resolution_m"].as<double>();
  }
  if (root["keyframe_filter_size_m"]) {
    meta.keyframe_filter_size_m = root["keyframe_filter_size_m"].as<double>();
  }
  const YAML::Node sc = root["scan_context"];
  if (!sc) return;
  if (sc["num_rings"]) meta.scan_context.num_rings = sc["num_rings"].as<int>();
  if (sc["num_sectors"]) meta.scan_context.num_sectors = sc["num_sectors"].as<int>();
  if (sc["max_radius_m"]) meta.scan_context.max_radius = sc["max_radius_m"].as<double>();
  if (sc["lidar_height_m"]) meta.scan_context.lidar_height = sc["lidar_height_m"].as<double>();
}

// Descriptors and poses only mean anything together, so a half-present or
// mismatched pair degrades the map to cloud-only rather than being trusted.
bool loadRelocalization(const std::filesystem::path& root, const LoadOptions& options,
                        const MapMeta& meta, std::optional<Relocalization>& out,
                        std::string* reason) {
  const auto poses_path = root / kPosesFile;
  const auto descriptor_path = root / kDescriptorFile;
  if (!std::filesystem::exists(poses_path) || !std::filesystem::exists(descriptor_path)) {
    *reason = "no descriptor database beside the cloud";
    return false;
  }

  Relocalization reloc;
  // Geometry is overwritten by load(); this carries the query-side settings.
  ScanContext::Config config = options.scan_context;
  config.num_rings = meta.scan_context.num_rings;
  config.num_sectors = meta.scan_context.num_sectors;
  config.max_radius = meta.scan_context.max_radius;
  config.lidar_height = meta.scan_context.lidar_height;
  reloc.scan_context = ScanContext(config);

  if (!reloc.scan_context.load(descriptor_path.string())) {
    *reason = "scan_context.bin could not be read";
    return false;
  }
  if (!loadPoses(poses_path.string(), reloc.poses, reloc.stamps)) {
    *reason = "poses_tum.txt could not be read";
    return false;
  }
  // Both lists are append-only records of the same keyframe sequence, so their
  // common prefix is always a valid pair even when the writer caught them
  // mid-append. Keep that prefix rather than discarding a usable database.
  if (reloc.poses.size() != reloc.scan_context.size()) {
    LOG(W, "Bundle has " << reloc.poses.size() << " poses but " << reloc.scan_context.size()
                         << " descriptors; using the common prefix.");
    const size_t common = std::min(reloc.poses.size(), reloc.scan_context.size());
    reloc.poses.resize(common);
    reloc.stamps.resize(common);
    reloc.scan_context.truncate(common);
  }

  out = std::move(reloc);
  return true;
}

}  // namespace

bool loadPriorMap(const std::string& path, const LoadOptions& options, PriorMap& map,
                  std::string* message) {
  const auto fail = [message](const std::string& text) {
    if (message) *message = text;
    return false;
  };

  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) return fail("No such prior map: " + path);

  if (!std::filesystem::is_directory(path, ec)) {
    // Bare PCD: a cloud and nothing else, so localization needs an initial pose.
    if (!loadCloud(path, options.voxel_size_m, map.cloud)) {
      return fail("Failed to read point cloud " + path);
    }
    logExtent(*map.cloud);
    map.meta.num_points = map.cloud->size();
    if (message) *message = "Loaded cloud-only prior map from " + path;
    return true;
  }

  const std::filesystem::path root(path);
  if (!loadCloud((root / kCloudFile).string(), options.voxel_size_m, map.cloud)) {
    return fail("Failed to read " + (root / kCloudFile).string());
  }
  logExtent(*map.cloud);

  if (std::filesystem::exists(root / kMetaFile)) {
    try {
      loadMeta((root / kMetaFile).string(), map.meta);
    } catch (const std::exception& e) {
      LOG(W, "Ignoring unreadable " << (root / kMetaFile).string() << ": " << e.what());
    }
  }

  std::string reason;
  if (loadRelocalization(root, options, map.meta, map.reloc, &reason)) {
    const auto& sc = map.reloc->scan_context.config();
    LOG(I, "Relocalization available: " << map.reloc->poses.size() << " keyframes, descriptor "
                                        << sc.num_rings << "x" << sc.num_sectors << " (radius "
                                        << sc.max_radius << " m, height " << sc.lidar_height
                                        << " m).");
    if (message) *message = "Loaded prior map bundle from " + path;
  } else {
    LOG(W, "Prior map loaded without relocalization (" << reason
                                                       << "); an initial pose is required.");
    if (message) *message = "Loaded cloud-only prior map from " + path + " (" + reason + ")";
  }
  return true;
}

bool saveMapBundle(const std::string& dir, const MapCloud& cloud,
                   const std::vector<Transform>& poses, const std::vector<uint64_t>& stamps,
                   const ScanContext::Archive& descriptors, const MapMeta& meta,
                   std::string* message) {
  const auto fail = [message](const std::string& text) {
    if (message) *message = text;
    return false;
  };

  if (cloud.empty()) return fail("Nothing to save: the map cloud is empty.");

  // Relocalization reads pose i and descriptor i as the same keyframe, so the
  // bundle is written to the common prefix. The caller snapshots the two under
  // different locks and can legitimately catch a keyframe mid-append.
  const size_t count =
      std::min({poses.size(), stamps.size(), descriptors.descriptors.size()});
  if (poses.size() != stamps.size() || poses.size() != descriptors.descriptors.size()) {
    LOG(W, "Snapshot mismatch: " << poses.size() << " poses, " << stamps.size() << " stamps, "
                                 << descriptors.descriptors.size() << " descriptors; saving "
                                 << count << ".");
  }
  if (count == 0) return fail("Nothing to save: no keyframes.");

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) return fail("Cannot create " + dir + ": " + ec.message());

  const std::filesystem::path root(dir);

  if (pcl::io::savePCDFileBinary((root / kCloudFile).string(), cloud) != 0) {
    return fail("Failed to write cloud.pcd");
  }

  std::ofstream poses_file((root / kPosesFile).string());
  if (!poses_file) return fail("Failed to write poses_tum.txt");
  poses_file << std::fixed << std::setprecision(9);
  for (size_t i = 0; i < count; ++i) {
    const Point t = poses[i].translation();
    const Quaternion q = poses[i].quaternion();
    poses_file << nsToS(stamps[i]) << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x()
               << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }

  ScanContext::Archive trimmed = descriptors;
  trimmed.descriptors.resize(count);
  if (!ScanContext::saveArchive(trimmed, (root / kDescriptorFile).string())) {
    return fail("Failed to write scan_context.bin");
  }

  std::ofstream meta_file((root / kMetaFile).string());
  if (!meta_file) return fail("Failed to write meta.yaml");
  const auto& sc = meta.scan_context;
  meta_file << "frame: \"" << meta.frame << "\"\n"
            << "num_keyframes: " << count << "\n"
            << "num_points: " << cloud.size() << "\n"
            << "map_save_resolution_m: " << meta.map_save_resolution_m << "\n"
            << "keyframe_filter_size_m: " << meta.keyframe_filter_size_m << "\n"
            << "scan_context:\n"
            << "  num_rings: " << sc.num_rings << "\n"
            << "  num_sectors: " << sc.num_sectors << "\n"
            << "  max_radius_m: " << sc.max_radius << "\n"
            << "  lidar_height_m: " << sc.lidar_height << "\n";

  if (message) {
    *message = "Saved " + std::to_string(cloud.size()) + " points from " +
               std::to_string(count) + " keyframes to " + root.string();
  }
  return true;
}

}  // namespace bievr
