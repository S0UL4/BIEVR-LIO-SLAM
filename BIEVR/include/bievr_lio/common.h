#ifndef BIEVR_LIO_COMMON_H_
#define BIEVR_LIO_COMMON_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace bievr {

using Rotation = Eigen::Matrix3d;
using Quaternion = Eigen::Quaternion<double>;
using Point = Eigen::Vector3d;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using M4 = Eigen::Matrix4d;
using V3Map = Eigen::Map<const V3>;
using QuatMap = Eigen::Map<const Quaternion>;

// xyz + per-point time offset (row 3) + intensity (row 4). The intensity rides
// along as the last row so it survives range filtering and undistortion without
// any of those stages having to touch it.
struct StampedIntensityPoint : public Eigen::Matrix<double, 5, 1> {
  using Base = Eigen::Matrix<double, 5, 1>;
  using Base::Base;
  StampedIntensityPoint(const Eigen::Vector3d&) = delete;  // forbid implicit conv
};
// xyz + intensity (row 3). Only used to publish clouds with an intensity channel.
struct IntensityPoint : public Eigen::Vector4d {
  using Eigen::Vector4d::Vector4d;
  IntensityPoint(const Eigen::Vector3d&) = delete;  // forbid implicit conv
};
// A single per-point channel (time or intensity) as an owning row vector.
using Intensities = Eigen::Matrix<double, 1, Eigen::Dynamic>;
// Non-owning, read-only view onto one such row. The channel lives as a strided row
// of a column-major cloud (inner stride == the matrix's row count), so a plain
// Eigen::Ref would silently copy it; an explicit strided Map references it in
// place. Valid only while the source cloud outlives the view.
using RowView = Eigen::Map<const Intensities, 0, Eigen::InnerStride<>>;
using TimeView = RowView;
using IntensityView = RowView;

template <typename T>
inline constexpr int dim_v = T::RowsAtCompileTime;

template <typename PointT>
class PointcloudBase {
 public:
  static constexpr int kDim = dim_v<PointT>;
  using PointcloudData = Eigen::Matrix<double, kDim, Eigen::Dynamic>;
  using PointData = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  PointcloudBase() = default;
  explicit PointcloudBase(PointcloudData pointcloud) : data_(std::move(pointcloud)) {}

  PointcloudBase operator+(const PointcloudBase& other) const {
    PointcloudBase result;
    result.data_.resize(kDim, this->size() + other.size());
    result.data_.leftCols(this->size()) = this->data_;
    result.data_.rightCols(other.size()) = other.data_;
    return result;
  }

  operator PointcloudBase<Point>() const {
    // copy only the top 3 rows (spatial coordinates)
    return PointcloudBase<Point>(this->data_.topRows(3).eval());
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(size_t n_points) { data_.conservativeResize(kDim, n_points); }
  void clear() { data_.resize(kDim, 0); }

  Eigen::Block<PointcloudData> points() { return data_.topRows(3); }
  const PointData& points() const { return data_.topRows(3); }

  PointcloudData& data() { return data_; }
  const PointcloudData& data() const { return data_; }

  typename PointcloudData::ColXpr operator[](Eigen::Index point_index) {
    return data_.col(point_index);
  }

  typename PointcloudData::ConstColXpr operator[](Eigen::Index point_index) const {
    return data_.col(point_index);
  }

  void push_back(const PointT& point) {
    const int cols = data_.cols();
    resize(cols + 1);
    data_.col(cols) = point;
  }

 protected:
  PointcloudData data_;
};

using Pointcloud = PointcloudBase<Point>;

struct ImuMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp;
  V3 acc;
  V3 gyro;
};

class Transform : public Eigen::Isometry3d {
  using Eigen::Isometry3d::Isometry3d;

 public:
  Transform() : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {}

  Transform(const Rotation& rot, const Point& trans)
      : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {
    linear() = rot;
    translation() = trans;
  }

  Transform(const Quaternion& rot, const Point& trans) : Eigen::Isometry3d(rot) {
    translation() = trans;
  }
  Transform(const Point& trans, const Rotation& rot) {
    linear() = rot;
    translation() = trans;
  }

  Transform(const Point& trans, const Quaternion& rot) : Eigen::Isometry3d(rot) {
    translation() = trans;
  }

  Transform(const Point& trans) : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {
    translation() = trans;
  }

  Transform(const double x, const double y, const double z)
      : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {
    translation() << x, y, z;
  }

  Transform(const Eigen::Isometry3d& iso) : Eigen::Isometry3d(iso) {}
  Rotation rotation() const { return linear(); }
  Quaternion quaternion() const { return Quaternion(linear()); }
};

struct State {
  uint64_t id;
  Quaternion quat;
  V3 p;
  V3 v;
};

// Bundles a pose with its body-frame twist for publishing odometry messages.
struct Odometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Transform pose;       // T_W_B (pose in the map/world frame)
  V3 linear_velocity;   // linear velocity expressed in the body frame
  V3 angular_velocity;  // angular velocity expressed in the body frame
};

// One entry of a published Path: a map/world-frame pose with its own stamp.
struct StampedPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp;
  Transform pose;  // T_W_B
};

// Trajectory of stamped poses, accumulated for publishing as a path message.
struct Path {
  std::vector<StampedPose> poses;
};

template <typename PointT>
inline PointcloudBase<PointT> operator*(const Transform& T, PointcloudBase<PointT> P) {
  P.points() = T * P.points().colwise().homogeneous();
  return P;
}

// Transform only the spatial coordinates of any cloud into a fresh Pointcloud,
// dropping the extra channels (time/intensity). Reads xyz in place from the source
// and performs a single 3xN write, so the source's per-point channels are never
// rewritten (their values can instead be carried along as views).
template <typename PointT>
inline Pointcloud transformPoints(const Transform& T, const PointcloudBase<PointT>& P) {
  Pointcloud out;
  out.resize(P.size());
  out.data().noalias() = T.linear() * P.data().topRows(3);
  out.data().colwise() += T.translation();
  return out;
}

class StampedIntensityPointcloud : public PointcloudBase<StampedIntensityPoint> {
 public:
  // Layout: rows 0-2 = xyz, row 3 = per-point time offset, row 4 = intensity.
  static constexpr int kTimeRow = 3;
  static constexpr int kIntensityRow = 4;
  // Non-const overloads return a writable block view (used while filling the cloud
  // from a message). Const overloads return a zero-copy strided view onto the row,
  // which downstream stages read without rewriting the data.
  Eigen::Block<PointcloudData> times() { return data_.middleRows(kTimeRow, 1); }
  TimeView times() const { return rowView(kTimeRow); }
  Eigen::Block<PointcloudData> intensities() { return data_.bottomRows(1); }
  IntensityView intensities() const { return rowView(kIntensityRow); }

  StampedIntensityPointcloud operator+(const StampedIntensityPointcloud& other) const = delete;
  uint64_t stamp;
  uint64_t end_stamp;

 private:
  RowView rowView(int row) const {
    return RowView(data_.data() + row, data_.cols(), Eigen::InnerStride<>(data_.outerStride()));
  }
};

// Spatial cloud with an intensity channel (row 3), assembled only for publishing.
class IntensityPointcloud : public PointcloudBase<IntensityPoint> {
 public:
  IntensityPointcloud() = default;
  // Combine spatial points with a matching per-point intensity row. Both inputs
  // must have the same number of columns and be in the same point order. The
  // intensity is taken as a non-owning view, so this is the only place its values
  // are copied (into the cloud assembled for publishing).
  IntensityPointcloud(const Pointcloud& points, const IntensityView& intensities) {
    resize(points.size());
    data_.topRows(3) = points.data();
    data_.bottomRows(1) = intensities;
  }

  // Non-const returns a writable block view; const returns a zero-copy strided
  // view onto the intensity row (see StampedIntensityPointcloud).
  Eigen::Block<PointcloudData> intensities() { return data_.bottomRows(1); }
  IntensityView intensities() const {
    return IntensityView(data_.data() + (kDim - 1), data_.cols(),
                         Eigen::InnerStride<>(data_.outerStride()));
  }
};

struct Header {
  uint64_t stamp;
  uint32_t seq;
  std::string frame;
};

inline uint64_t ExpandBits(int32_t v) {
  uint64_t x = static_cast<uint64_t>(v + (1 << 20)) & 0x1fffff;
  x = (x | (x << 32)) & 0x1f00000000ffffULL;
  x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
  x = (x | (x << 8)) & 0x100f00f00f00f00fULL;
  x = (x | (x << 4)) & 0x10c30c30c30c30c3ULL;
  x = (x | (x << 2)) & 0x1249249249249249ULL;
  return x;
}

inline size_t hashIndexVoxel(const Eigen::Vector3i& voxel) {
  uint64_t morton =
      ExpandBits(voxel.x()) | (ExpandBits(voxel.y()) << 1) | (ExpandBits(voxel.z()) << 2);
  return static_cast<std::size_t>(morton);
}

void calculateRanges(const Pointcloud& cloud, std::vector<double>& ranges);

}  // namespace bievr

#endif  // BIEVR_LIO_COMMON_H_