#include "bievr_scancontext/scan_context.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>

namespace bievr {
namespace {

constexpr char kMagic[4] = {'B', 'S', 'C', 'D'};
constexpr uint32_t kVersion = 1;

// Azimuth in [0, 360). Equivalent to SC-A-LOAM's four-quadrant xy2theta, but
// without its fallthrough and its 0/0 at the origin.
double xy2theta(double x, double y) {
  double deg = std::atan2(y, x) * 180.0 / M_PI;
  return deg < 0.0 ? deg + 360.0 : deg;
}

Eigen::MatrixXd circshift(const Eigen::MatrixXd& mat, int shift) {
  if (shift == 0) return mat;
  Eigen::MatrixXd out(mat.rows(), mat.cols());
  for (int col = 0; col < mat.cols(); ++col) {
    out.col((col + shift) % mat.cols()) = mat.col(col);
  }
  return out;
}

std::vector<float> toStdVec(const Eigen::MatrixXd& mat) {
  std::vector<float> vec(mat.size());
  for (Eigen::Index i = 0; i < mat.size(); ++i) vec[i] = static_cast<float>(mat.data()[i]);
  return vec;
}

}  // namespace

ScanContext::ScanContext(Config config) : config_(std::move(config)) {}

Eigen::MatrixXd ScanContext::makeDescriptor(const ScanContextCloud& cloud) const {
  // Empty bins must read 0 so directDistance() can skip them, so build with a
  // sentinel and flatten it afterwards.
  constexpr double kNoPoint = -1000.0;
  Eigen::MatrixXd desc =
      Eigen::MatrixXd::Constant(config_.num_rings, config_.num_sectors, kNoPoint);

  for (Eigen::Index i = 0; i < cloud.cols(); ++i) {
    const double x = cloud(0, i);
    const double y = cloud(1, i);
    const double z = cloud(2, i) + config_.lidar_height;

    const double range = std::sqrt(x * x + y * y);
    if (range > config_.max_radius) continue;

    const int ring = std::clamp(
        static_cast<int>(std::ceil(range / config_.max_radius * config_.num_rings)), 1,
        config_.num_rings);
    const int sector = std::clamp(
        static_cast<int>(std::ceil(xy2theta(x, y) / 360.0 * config_.num_sectors)), 1,
        config_.num_sectors);

    desc(ring - 1, sector - 1) = std::max(desc(ring - 1, sector - 1), z);
  }

  return (desc.array() == kNoPoint).select(0.0, desc);
}

Eigen::MatrixXd ScanContext::ringKey(const Eigen::MatrixXd& desc) const {
  return desc.rowwise().mean();
}

Eigen::MatrixXd ScanContext::sectorKey(const Eigen::MatrixXd& desc) const {
  return desc.colwise().mean();
}

double ScanContext::directDistance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const {
  int num_eff_cols = 0;
  double sum_similarity = 0.0;
  for (Eigen::Index col = 0; col < a.cols(); ++col) {
    const double norm_a = a.col(col).norm();
    const double norm_b = b.col(col).norm();
    if (norm_a == 0.0 || norm_b == 0.0) continue;  // a sector unseen in either scan

    sum_similarity += a.col(col).dot(b.col(col)) / (norm_a * norm_b);
    ++num_eff_cols;
  }
  if (num_eff_cols == 0) return 1.0;
  return 1.0 - sum_similarity / num_eff_cols;
}

int ScanContext::alignBySectorKey(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) const {
  int best_shift = 0;
  double min_norm = std::numeric_limits<double>::max();
  for (Eigen::Index shift = 0; shift < a.cols(); ++shift) {
    const double norm = (a - circshift(b, static_cast<int>(shift))).norm();
    if (norm < min_norm) {
      min_norm = norm;
      best_shift = static_cast<int>(shift);
    }
  }
  return best_shift;
}

std::pair<double, int> ScanContext::distance(const Eigen::MatrixXd& a,
                                             const Eigen::MatrixXd& b) const {
  // Coarse yaw from the sector keys, then an exact search in a window around it.
  const int coarse_shift = alignBySectorKey(sectorKey(a), sectorKey(b));
  const int num_cols = static_cast<int>(a.cols());
  const int radius = static_cast<int>(std::round(0.5 * config_.search_ratio * num_cols));

  std::vector<int> shifts{coarse_shift};
  for (int i = 1; i <= radius; ++i) {
    shifts.push_back((coarse_shift + i + num_cols) % num_cols);
    shifts.push_back((coarse_shift - i + num_cols) % num_cols);
  }

  int best_shift = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (const int shift : shifts) {
    const double dist = directDistance(a, circshift(b, shift));
    if (dist < min_dist) {
      min_dist = dist;
      best_shift = shift;
    }
  }
  return {min_dist, best_shift};
}

ScanContext::Match ScanContext::search(KeyTree& tree, const std::vector<float>& key,
                                       const Eigen::MatrixXd& desc,
                                       size_t num_searchable) const {
  Match match;
  if (num_searchable == 0) return match;

  // Upstream asks for num_candidates unconditionally and reads the result array
  // in full; with a database smaller than that, the tail holds uninitialised
  // indices. Clamp, then trust nanoflann's reported count.
  const size_t k = std::min<size_t>(config_.num_candidates, num_searchable);
  std::vector<size_t> indices(k);
  std::vector<float> dists_sqr(k);

  nanoflann::KNNResultSet<float> result(k);
  result.init(indices.data(), dists_sqr.data());
  tree.index->findNeighbors(result, key.data(), nanoflann::SearchParams(10));

  const size_t num_found = result.size();
  int best_shift = 0;
  for (size_t i = 0; i < num_found; ++i) {
    const auto [dist, shift] = distance(desc, descriptors_[indices[i]]);
    if (dist < match.distance) {
      match.distance = dist;
      match.index = static_cast<int>(indices[i]);
      best_shift = shift;
    }
  }

  const double sector_angle = 360.0 / config_.num_sectors;
  match.yaw_diff_rad = static_cast<float>(best_shift * sector_angle * M_PI / 180.0);
  if (match.distance >= config_.dist_threshold) match.index = -1;
  return match;
}

void ScanContext::append(Eigen::MatrixXd desc) {
  ring_keys_.push_back(toStdVec(ringKey(desc)));
  descriptors_.push_back(std::move(desc));
}

void ScanContext::add(const ScanContextCloud& cloud_body) {
  append(makeDescriptor(cloud_body));
}

ScanContext::Match ScanContext::detectLoop() {
  const int num_exclude = config_.num_exclude_recent;
  if (static_cast<int>(descriptors_.size()) < num_exclude + 1) return Match{};

  const size_t num_searchable = descriptors_.size() - num_exclude;
  if (tree_counter_ % config_.tree_making_period == 0) {
    search_keys_.assign(ring_keys_.begin(), ring_keys_.end() - num_exclude);
    tree_ = std::make_unique<KeyTree>(config_.num_rings, search_keys_, 10 /* max leaf */);
  }
  ++tree_counter_;

  return search(*tree_, ring_keys_.back(), descriptors_.back(), num_searchable);
}

ScanContext::Match ScanContext::query(const ScanContextCloud& cloud_body) {
  if (descriptors_.empty()) return Match{};

  if (!batch_tree_ || batch_size_ != descriptors_.size()) {
    batch_keys_ = ring_keys_;
    batch_tree_ = std::make_unique<KeyTree>(config_.num_rings, batch_keys_, 10 /* max leaf */);
    batch_size_ = descriptors_.size();
  }

  const Eigen::MatrixXd desc = makeDescriptor(cloud_body);
  return search(*batch_tree_, toStdVec(ringKey(desc)), desc, descriptors_.size());
}

bool ScanContext::save(const std::string& path) const { return saveArchive(archive(), path); }

bool ScanContext::saveArchive(const Archive& archive, const std::string& path) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cerr << "ScanContext: cannot write " << path << std::endl;
    return false;
  }

  const int32_t rings = archive.config.num_rings;
  const int32_t sectors = archive.config.num_sectors;
  const uint64_t count = archive.descriptors.size();

  out.write(kMagic, sizeof(kMagic));
  out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
  out.write(reinterpret_cast<const char*>(&rings), sizeof(rings));
  out.write(reinterpret_cast<const char*>(&sectors), sizeof(sectors));
  out.write(reinterpret_cast<const char*>(&archive.config.max_radius), sizeof(double));
  out.write(reinterpret_cast<const char*>(&archive.config.lidar_height), sizeof(double));
  out.write(reinterpret_cast<const char*>(&count), sizeof(count));

  // Ring keys are derived, so only the descriptors are stored.
  for (const auto& desc : archive.descriptors) {
    out.write(reinterpret_cast<const char*>(desc.data()), desc.size() * sizeof(double));
  }
  return out.good();
}

bool ScanContext::load(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "ScanContext: cannot read " << path << std::endl;
    return false;
  }

  char magic[4];
  uint32_t version = 0;
  int32_t rings = 0;
  int32_t sectors = 0;
  double max_radius = 0.0;
  double lidar_height = 0.0;
  uint64_t count = 0;

  in.read(magic, sizeof(magic));
  in.read(reinterpret_cast<char*>(&version), sizeof(version));
  in.read(reinterpret_cast<char*>(&rings), sizeof(rings));
  in.read(reinterpret_cast<char*>(&sectors), sizeof(sectors));
  in.read(reinterpret_cast<char*>(&max_radius), sizeof(max_radius));
  in.read(reinterpret_cast<char*>(&lidar_height), sizeof(lidar_height));
  in.read(reinterpret_cast<char*>(&count), sizeof(count));

  if (!in || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    std::cerr << "ScanContext: " << path << " is not a descriptor database" << std::endl;
    return false;
  }
  if (version != kVersion) {
    std::cerr << "ScanContext: unsupported version " << version << " in " << path << std::endl;
    return false;
  }

  // Descriptors are only comparable under the geometry they were built with, so
  // the file's geometry wins over whatever was configured.
  if (rings != config_.num_rings || sectors != config_.num_sectors ||
      lidar_height != config_.lidar_height || max_radius != config_.max_radius) {
    std::cerr << "ScanContext: adopting geometry from " << path << " (rings " << rings
              << ", sectors " << sectors << ", max_radius " << max_radius << ", lidar_height "
              << lidar_height << ")" << std::endl;
  }
  config_.num_rings = rings;
  config_.num_sectors = sectors;
  config_.max_radius = max_radius;
  config_.lidar_height = lidar_height;

  descriptors_.clear();
  ring_keys_.clear();
  descriptors_.reserve(count);
  ring_keys_.reserve(count);

  for (uint64_t i = 0; i < count; ++i) {
    Eigen::MatrixXd desc(rings, sectors);
    in.read(reinterpret_cast<char*>(desc.data()), desc.size() * sizeof(double));
    if (!in) {
      std::cerr << "ScanContext: " << path << " truncated at descriptor " << i << std::endl;
      descriptors_.clear();
      ring_keys_.clear();
      return false;
    }
    append(std::move(desc));
  }

  tree_.reset();
  tree_counter_ = 0;
  batch_tree_.reset();
  batch_size_ = 0;
  return true;
}

void ScanContext::truncate(size_t count) {
  if (count >= descriptors_.size()) return;
  descriptors_.resize(count);
  ring_keys_.resize(count);
  // Both trees index the vectors that just shrank.
  tree_.reset();
  tree_counter_ = 0;
  batch_tree_.reset();
  batch_size_ = 0;
}

}  // namespace bievr
