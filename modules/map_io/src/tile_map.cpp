#include "bievr_map_io/tile_map.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>
#include <vector>

#include "bievr_lio/log++.h"

namespace bievr {
namespace {

constexpr const char* kIndexFile = "index.yaml";
constexpr const char* kOverviewFile = "overview.pcd";

// Raw points held at once while tiling. The source is walked again for each
// batch, so this trades passes against peak memory: 20M points is ~240 MB and
// four passes over a 78M-point map.
constexpr size_t kBatchPointBudget = 20000000;

struct PcdLayout {
  size_t data_offset = 0;
  size_t point_step = 0;
  size_t x_offset = 0, y_offset = 0, z_offset = 0;
  size_t num_points = 0;
};

// Parses just enough of a PCD header to walk `DATA binary` float32 xyz in place.
// Anything else -- ascii, binary_compressed, non-float xyz -- is rejected so the
// caller falls back to a full load. Extra fields (intensity, ring) are fine:
// only the offsets of x/y/z matter.
bool readBinaryLayout(const std::string& path, PcdLayout& layout) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return false;

  std::vector<std::string> fields;
  std::vector<size_t> sizes, counts;
  std::vector<char> types;
  size_t points = 0;
  bool binary = false;

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    std::string token;
    stream >> token;
    if (token == "FIELDS") {
      for (std::string name; stream >> name;) fields.push_back(name);
    } else if (token == "SIZE") {
      for (size_t size; stream >> size;) sizes.push_back(size);
    } else if (token == "TYPE") {
      for (std::string type; stream >> type;) types.push_back(type.empty() ? '?' : type[0]);
    } else if (token == "COUNT") {
      for (size_t count; stream >> count;) counts.push_back(count);
    } else if (token == "POINTS") {
      stream >> points;
    } else if (token == "DATA") {
      std::string format;
      stream >> format;
      binary = format == "binary";
      layout.data_offset = static_cast<size_t>(file.tellg());
      break;
    }
  }
  if (!binary || points == 0 || fields.empty()) return false;
  if (fields.size() != sizes.size() || fields.size() != types.size()) return false;
  if (counts.empty()) counts.assign(fields.size(), 1);
  if (counts.size() != fields.size()) return false;

  size_t offset = 0;
  bool have[3] = {false, false, false};
  for (size_t i = 0; i < fields.size(); ++i) {
    const size_t width = sizes[i] * counts[i];
    const bool f32 = types[i] == 'F' && sizes[i] == 4 && counts[i] == 1;
    if (fields[i] == "x" && f32) {
      layout.x_offset = offset;
      have[0] = true;
    } else if (fields[i] == "y" && f32) {
      layout.y_offset = offset;
      have[1] = true;
    } else if (fields[i] == "z" && f32) {
      layout.z_offset = offset;
      have[2] = true;
    }
    offset += width;
  }
  if (!have[0] || !have[1] || !have[2]) return false;

  layout.point_step = offset;
  layout.num_points = points;
  return true;
}

// Walks the source cloud, once per call to forEach(). mmap when the layout
// allows it, so tiling never holds the whole file; otherwise a full PCL load,
// which is correct but needs the memory.
class SourceReader {
 public:
  ~SourceReader() {
    if (data_) ::munmap(const_cast<uint8_t*>(data_), bytes_);
    if (fd_ >= 0) ::close(fd_);
  }

  bool open(const std::string& path, std::string* message) {
    PcdLayout layout;
    if (readBinaryLayout(path, layout)) {
      fd_ = ::open(path.c_str(), O_RDONLY);
      struct stat info {};
      if (fd_ >= 0 && ::fstat(fd_, &info) == 0) {
        bytes_ = static_cast<size_t>(info.st_size);
        const size_t needed = layout.data_offset + layout.num_points * layout.point_step;
        void* mapped = needed <= bytes_
                           ? ::mmap(nullptr, bytes_, PROT_READ, MAP_SHARED, fd_, 0)
                           : MAP_FAILED;
        if (mapped != MAP_FAILED) {
          ::madvise(mapped, bytes_, MADV_SEQUENTIAL);
          data_ = static_cast<const uint8_t*>(mapped);
          layout_ = layout;
          num_points_ = layout.num_points;
          return true;
        }
      }
      if (fd_ >= 0) ::close(fd_);
      fd_ = -1;
    }

    cloud_.reset(new MapCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud_) != 0 || cloud_->empty()) {
      if (message) *message = "cannot read " + path;
      return false;
    }
    LOG(I, "Tiling " << path << " through a full load; it is not an uncompressed binary PCD.");
    num_points_ = cloud_->size();
    return true;
  }

  size_t size() const { return num_points_; }

  template <typename Fn>
  void forEach(Fn&& visit) const {
    if (data_) {
      const uint8_t* point = data_ + layout_.data_offset;
      for (size_t i = 0; i < layout_.num_points; ++i, point += layout_.point_step) {
        float x, y, z;
        std::memcpy(&x, point + layout_.x_offset, sizeof(float));
        std::memcpy(&y, point + layout_.y_offset, sizeof(float));
        std::memcpy(&z, point + layout_.z_offset, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        visit(x, y, z);
      }
      return;
    }
    for (const auto& p : cloud_->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      visit(p.x, p.y, p.z);
    }
  }

 private:
  int fd_ = -1;
  const uint8_t* data_ = nullptr;
  size_t bytes_ = 0;
  PcdLayout layout_;
  MapCloud::Ptr cloud_;
  size_t num_points_ = 0;
};

// Seconds since the Unix epoch. file_time_type's own epoch is unspecified, and
// this value lands in a file a human may well read.
int64_t mtimeOf(const std::filesystem::path& path) {
  std::error_code ec;
  const auto stamp = std::filesystem::last_write_time(path, ec);
  if (ec) return 0;
  const auto system = std::chrono::file_clock::to_sys(stamp);
  return std::chrono::duration_cast<std::chrono::seconds>(system.time_since_epoch()).count();
}

int64_t sizeOf(const std::filesystem::path& path) {
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  return ec ? 0 : static_cast<int64_t>(size);
}

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

}  // namespace

TileKey TileMap::keyAt(double x, double y) const {
  TileKey key;
  // std::floor, never a cast: truncation toward zero makes the origin tile
  // double width, which is the book's bug.
  key.x = static_cast<int64_t>(std::floor(x / config_.tile_size_m));
  key.y = static_cast<int64_t>(std::floor(y / config_.tile_size_m));
  return key;
}

std::vector<TileKey> TileMap::ringAround(const TileKey& centre, int ring) const {
  std::vector<TileKey> keys;
  for (int64_t dx = -ring; dx <= ring; ++dx) {
    for (int64_t dy = -ring; dy <= ring; ++dy) {
      const TileKey key{centre.x + dx, centre.y + dy};
      if (has(key)) keys.push_back(key);
    }
  }
  return keys;
}

std::string TileMap::tilePath(const std::string& dir, const TileKey& key) const {
  std::ostringstream name;
  name << dir << "/" << key.x << "_" << key.y << ".pcd";
  return name.str();
}

MapCloud::Ptr TileMap::load(const TileKey& key) const {
  if (!has(key)) return nullptr;
  MapCloud::Ptr cloud(new MapCloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(tilePath(dir_, key), *cloud) != 0) {
    LOG(W, "Cannot read tile " << tilePath(dir_, key) << ".");
    return nullptr;
  }
  return cloud;
}

bool TileMap::readIndex(const std::string& dir, const std::string& source, std::string* message) {
  const std::filesystem::path index_path = std::filesystem::path(dir) / kIndexFile;
  std::error_code ec;
  if (!std::filesystem::exists(index_path, ec)) {
    if (message) *message = "no cache yet";
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(index_path.string());
  } catch (const std::exception& e) {
    if (message) *message = std::string("unreadable index: ") + e.what();
    return false;
  }

  try {
    // The cache is keyed on both what it was built from and how, so a replaced
    // map or a changed leaf rebuilds and nothing else does.
    if (root["source"]["path"].as<std::string>() != source) {
      if (message) *message = "built from a different file";
      return false;
    }
    if (root["source"]["size"].as<int64_t>() != sizeOf(source) ||
        root["source"]["mtime"].as<int64_t>() != mtimeOf(source)) {
      if (message) *message = "source cloud changed";
      return false;
    }
    if (!near(root["tile_size_m"].as<double>(), config_.tile_size_m) ||
        !near(root["voxel_size_m"].as<double>(), config_.voxel_size_m)) {
      if (message) *message = "built with different parameters";
      return false;
    }
    // The display leaf is not part of the key: it can be satisfied from the
    // tiles alone, so changing it must not cost a pass over the source.
    cached_overview_leaf_ = root["overview_voxel_size_m"].as<double>();

    tiles_.clear();
    index_.clear();
    num_points_ = 0;
    for (const auto& row : root["tiles"]) {
      // [gx, gy, points, minx, miny, minz, maxx, maxy, maxz]
      TileInfo info;
      info.key.x = row[0].as<int64_t>();
      info.key.y = row[1].as<int64_t>();
      info.num_points = row[2].as<size_t>();
      info.min = Eigen::Vector3f(row[3].as<float>(), row[4].as<float>(), row[5].as<float>());
      info.max = Eigen::Vector3f(row[6].as<float>(), row[7].as<float>(), row[8].as<float>());
      index_.emplace(info.key, tiles_.size());
      num_points_ += info.num_points;
      tiles_.push_back(info);
    }
  } catch (const std::exception& e) {
    if (message) *message = std::string("malformed index: ") + e.what();
    return false;
  }

  if (tiles_.empty()) {
    if (message) *message = "index lists no tiles";
    return false;
  }
  min_ = tiles_.front().min;
  max_ = tiles_.front().max;
  for (const TileInfo& info : tiles_) {
    min_ = min_.cwiseMin(info.min);
    max_ = max_.cwiseMax(info.max);
  }
  return true;
}

bool TileMap::writeIndex(const std::string& dir, const std::string& source) const {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "source" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "path" << YAML::Value << source;
  out << YAML::Key << "size" << YAML::Value << sizeOf(source);
  out << YAML::Key << "mtime" << YAML::Value << mtimeOf(source);
  out << YAML::EndMap;
  out << YAML::Key << "tile_size_m" << YAML::Value << config_.tile_size_m;
  out << YAML::Key << "voxel_size_m" << YAML::Value << config_.voxel_size_m;
  out << YAML::Key << "overview_voxel_size_m" << YAML::Value << cached_overview_leaf_;
  out << YAML::Key << "tiles" << YAML::Value << YAML::BeginSeq;
  for (const TileInfo& info : tiles_) {
    out << YAML::Flow << YAML::BeginSeq << info.key.x << info.key.y
        << static_cast<int64_t>(info.num_points) << info.min.x() << info.min.y() << info.min.z()
        << info.max.x() << info.max.y() << info.max.z() << YAML::EndSeq;
  }
  out << YAML::EndSeq << YAML::EndMap;

  // Through a temporary, so a crash here cannot leave an index that disagrees
  // with the tiles beside it.
  const std::filesystem::path final_path = std::filesystem::path(dir) / kIndexFile;
  const std::filesystem::path tmp_path(final_path.string() + ".tmp");
  {
    std::ofstream index(tmp_path.string());
    if (!index) return false;
    index << out.c_str() << "\n";
    if (!index) return false;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, final_path, ec);
  return !ec;
}

bool TileMap::rebuildOverview(const std::string& dir, const std::string& source) {
  LOG(I, "Rebuilding the map overview at " << config_.overview_voxel_size_m
                                           << " m from the existing tiles.");
  VoxelAccumulator accumulator(config_.overview_voxel_size_m);
  for (const TileInfo& info : tiles_) {
    const MapCloud::Ptr tile = load(info.key);
    if (tile) accumulator.add(*tile);
  }
  MapCloud::Ptr overview = accumulator.take();
  if (overview->empty()) return false;

  const std::filesystem::path final_path = std::filesystem::path(dir) / kOverviewFile;
  const std::filesystem::path tmp_path(final_path.string() + ".tmp");
  if (pcl::io::savePCDFileBinary(tmp_path.string(), *overview) != 0) return false;
  std::error_code ec;
  std::filesystem::rename(tmp_path, final_path, ec);
  if (ec) return false;

  cached_overview_leaf_ = config_.overview_voxel_size_m;
  overview_ = overview;
  if (!writeIndex(dir, source)) {
    LOG(W, "The overview was rebuilt but the index could not be updated; it will be redone.");
  }
  return true;
}

bool TileMap::build(const std::string& dir, const std::string& source, std::string* message) {
  const auto fail = [message](const std::string& text) {
    if (message) *message = text;
    return false;
  };

  SourceReader reader;
  if (!reader.open(source, message)) return false;

  // Pass one: how many points land in each tile. Only counts are held, so this
  // is bounded by the tile count rather than by the cloud.
  std::unordered_map<TileKey, size_t, TileKeyHash> counts;
  reader.forEach([&](float x, float y, float) { ++counts[keyAt(x, y)]; });
  if (counts.empty()) return fail("source cloud is empty");

  std::error_code ec;
  const std::filesystem::path tmp(dir + ".tmp-" + std::to_string(::getpid()));
  std::filesystem::remove_all(tmp, ec);
  if (!std::filesystem::create_directories(tmp, ec)) {
    return fail("cannot create " + tmp.string() + " (" + ec.message() + ")");
  }

  std::vector<TileKey> pending;
  pending.reserve(counts.size());
  for (const auto& entry : counts) pending.push_back(entry.first);
  std::sort(pending.begin(), pending.end(), [](const TileKey& a, const TileKey& b) {
    return a.x != b.x ? a.x < b.x : a.y < b.y;
  });

  tiles_.clear();
  index_.clear();
  num_points_ = 0;
  VoxelAccumulator overview(config_.overview_voxel_size_m);

  // Then one pass per batch of tiles, so the raw points held at once stay
  // bounded no matter how large the source is.
  size_t done = 0;
  size_t passes = 0;
  while (done < pending.size()) {
    std::unordered_map<TileKey, MapCloud::Ptr, TileKeyHash> batch;
    size_t budget = 0;
    while (done < pending.size() &&
           (batch.empty() || budget + counts[pending[done]] <= kBatchPointBudget)) {
      MapCloud::Ptr cloud(new MapCloud());
      cloud->reserve(counts[pending[done]]);
      budget += counts[pending[done]];
      batch.emplace(pending[done], cloud);
      ++done;
    }

    reader.forEach([&](float x, float y, float z) {
      const auto it = batch.find(keyAt(x, y));
      if (it != batch.end()) it->second->push_back(pcl::PointXYZ(x, y, z));
    });
    ++passes;

    for (auto& entry : batch) {
      const MapCloud::Ptr filtered = voxelDownsample(*entry.second, config_.voxel_size_m);
      entry.second.reset();  // the raw copy is dead the moment it is filtered
      if (filtered->empty()) continue;

      if (pcl::io::savePCDFileBinary(tilePath(tmp.string(), entry.first), *filtered) != 0) {
        std::filesystem::remove_all(tmp, ec);
        return fail("cannot write tiles into " + tmp.string());
      }
      overview.add(*filtered);

      TileInfo info;
      info.key = entry.first;
      info.num_points = filtered->size();
      info.min = filtered->points.front().getVector3fMap();
      info.max = info.min;
      for (const auto& p : filtered->points) {
        info.min = info.min.cwiseMin(p.getVector3fMap());
        info.max = info.max.cwiseMax(p.getVector3fMap());
      }
      index_.emplace(info.key, tiles_.size());
      num_points_ += info.num_points;
      tiles_.push_back(info);
    }
  }

  if (tiles_.empty()) {
    std::filesystem::remove_all(tmp, ec);
    return fail("every tile came out empty");
  }

  const MapCloud::Ptr overview_cloud = overview.take();
  if (pcl::io::savePCDFileBinary((tmp / kOverviewFile).string(), *overview_cloud) != 0) {
    std::filesystem::remove_all(tmp, ec);
    return fail("cannot write " + (tmp / kOverviewFile).string());
  }

  min_ = tiles_.front().min;
  max_ = tiles_.front().max;
  for (const TileInfo& info : tiles_) {
    min_ = min_.cwiseMin(info.min);
    max_ = max_.cwiseMax(info.max);
  }

  cached_overview_leaf_ = config_.overview_voxel_size_m;
  if (!writeIndex(tmp.string(), source)) {
    std::filesystem::remove_all(tmp, ec);
    return fail("cannot write " + (tmp / kIndexFile).string());
  }

  // Move into place only now that the cache is complete, and keep the old one
  // until the new one has landed: a build that dies here leaves the previous
  // cache intact rather than nothing at all.
  const std::filesystem::path final_dir(dir);
  const std::filesystem::path trash(dir + ".old-" + std::to_string(::getpid()));
  if (std::filesystem::exists(final_dir, ec)) {
    std::filesystem::rename(final_dir, trash, ec);
    if (ec) {
      std::filesystem::remove_all(tmp, ec);
      return fail("cannot move the old cache aside: " + ec.message());
    }
  }
  std::filesystem::rename(tmp, final_dir, ec);
  if (ec) {
    std::error_code restore;
    if (std::filesystem::exists(trash, restore)) {
      std::filesystem::rename(trash, final_dir, restore);
    }
    std::filesystem::remove_all(tmp, restore);
    return fail("cannot install the tile cache: " + ec.message());
  }
  std::filesystem::remove_all(trash, ec);

  overview_ = overview_cloud;
  LOG(I, "Tile cache built: " << tiles_.size() << " tiles, " << num_points_ << " points at "
                              << config_.voxel_size_m << " m, overview " << overview_cloud->size()
                              << " points, " << passes << " passes over the source.");
  return true;
}

bool TileMap::open(const std::string& source, const Config& config, std::string* message) {
  config_ = config;
  if (config_.tile_size_m <= 0.0) {
    if (message) *message = "tiling is disabled";
    return false;
  }

  std::error_code ec;
  const std::filesystem::path source_path = std::filesystem::absolute(source, ec);
  if (ec || !std::filesystem::exists(source_path, ec)) {
    if (message) *message = "no such cloud: " + source;
    return false;
  }
  dir_ = source_path.string() + ".tiles";

  std::string reason;
  if (readIndex(dir_, source_path.string(), &reason)) {
    bool have_overview = false;
    if (near(cached_overview_leaf_, config_.overview_voxel_size_m)) {
      overview_.reset(new MapCloud());
      const std::string overview_path = (std::filesystem::path(dir_) / kOverviewFile).string();
      have_overview = pcl::io::loadPCDFile<pcl::PointXYZ>(overview_path, *overview_) == 0;
      if (!have_overview) LOG(W, "Tile cache is missing " << overview_path << ".");
    } else {
      have_overview = rebuildOverview(dir_, source_path.string());
    }

    if (have_overview) {
      LOG(I, "Tile cache: " << tiles_.size() << " tiles, " << num_points_ << " points, overview "
                            << overview_->size() << " points, from " << dir_ << ".");
      return true;
    }
    reason = "the overview could not be produced";
  }

  LOG(I, "Building the tile cache in " << dir_ << " (" << reason << "); this runs once per map.");
  return build(dir_, source_path.string(), message);
}

}  // namespace bievr
