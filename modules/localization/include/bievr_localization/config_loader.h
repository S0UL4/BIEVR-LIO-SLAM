#ifndef BIEVR_LOCALIZATION_CONFIG_LOADER_H_
#define BIEVR_LOCALIZATION_CONFIG_LOADER_H_

// Parses the `localization:` section of the same YAML files bievr_lio loads, so
// the module configures itself without the core knowing it exists. Header-only;
// consumers need yaml-cpp on their include path (as with bievr_lio's loader).

#include <bievr_lio/config_loader.h>

#include <string>
#include <vector>

#include "bievr_localization/localizer.h"

namespace bievr {

struct LocalizationConfig {
  bool enable = false;
  Localizer::Config localizer;
  double publish_frequency = 50.0;  // Hz, the map->odom + fused pose broadcast
  double map_publish_frequency = 0.1;  // Hz, the prior map is heavy
};

inline bool loadLocalizationConfig(const std::vector<std::string>& yaml_paths,
                                   LocalizationConfig& config) {
  config_internal::MergedYaml yaml;
  for (const std::string& path : yaml_paths) {
    if (path.empty()) continue;
    try {
      yaml.add(YAML::LoadFile(path));
    } catch (const std::exception& e) {
      LOG(E, "Failed to load YAML config '" << path << "': " << e.what());
      return false;
    }
  }

  const std::string s = "localization";
  auto& c = config.localizer;

  config.enable = yaml.get<bool>(s, "enable", config.enable);
  if (!config.enable) return true;

  c.map_path = yaml.get<std::string>(s, "map_path", c.map_path);
  c.map_frame = yaml.get<std::string>(s, "map_frame", c.map_frame);
  c.map_voxel_size_m = yaml.get<double>(s, "map_voxel_size_m", c.map_voxel_size_m);
  c.scan_voxel_size_m = yaml.get<double>(s, "scan_voxel_size_m", c.scan_voxel_size_m);
  c.crop_radius_m = yaml.get<double>(s, "crop_radius_m", c.crop_radius_m);
  c.correction_frequency = yaml.get<double>(s, "correction_frequency", c.correction_frequency);

  c.coarse_scale = yaml.get<double>(s, "coarse_scale", c.coarse_scale);
  c.icp_max_correspondence_distance =
      yaml.get<double>(s, "icp_max_correspondence_distance", c.icp_max_correspondence_distance);
  c.icp_max_iterations = yaml.get<int>(s, "icp_max_iterations", c.icp_max_iterations);
  c.icp_transformation_epsilon =
      yaml.get<double>(s, "icp_transformation_epsilon", c.icp_transformation_epsilon);
  c.icp_euclidean_fitness_epsilon =
      yaml.get<double>(s, "icp_euclidean_fitness_epsilon", c.icp_euclidean_fitness_epsilon);
  c.fitness_threshold = yaml.get<double>(s, "fitness_threshold", c.fitness_threshold);

  auto& sc = c.scan_context;
  sc.num_candidates = yaml.get<int>(s, "sc_num_candidates", sc.num_candidates);
  sc.search_ratio = yaml.get<double>(s, "sc_search_ratio", sc.search_ratio);
  sc.dist_threshold = yaml.get<double>(s, "sc_dist_threshold", sc.dist_threshold);
  // num_exclude_recent is a mapping-side notion: relocalization searches the
  // whole database, so it is deliberately not exposed here.
  c.use_sc_yaw_guess = yaml.get<bool>(s, "use_sc_yaw_guess", c.use_sc_yaw_guess);
  c.max_consecutive_failures =
      yaml.get<int>(s, "max_consecutive_failures", c.max_consecutive_failures);

  config.publish_frequency = yaml.get<double>(s, "publish_frequency", config.publish_frequency);
  config.map_publish_frequency =
      yaml.get<double>(s, "map_publish_frequency", config.map_publish_frequency);
  return true;
}

}  // namespace bievr

#endif  // BIEVR_LOCALIZATION_CONFIG_LOADER_H_
