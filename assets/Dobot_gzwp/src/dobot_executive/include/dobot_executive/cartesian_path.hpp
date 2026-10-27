#ifndef DOBOT_EXECUTIVE__CARTESIAN_PATH_HPP_
#define DOBOT_EXECUTIVE__CARTESIAN_PATH_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dobot_executive
{

// ===========================================================================
// 1. STRATEGY PATTERN: ABSTRACT CARTESIAN GEOMETRIC PATH (SRP, OCP, LSP)
// ===========================================================================

/**
 * @brief Abstract Cartesian Path Strategy.
 * Maps normalized path parameter s \in [0, 1] to homogeneous SE(3) pose T(s).
 */
class ICartesianPath
{
public:
  virtual ~ICartesianPath() = default;

  [[nodiscard]] virtual std::string getName() const noexcept = 0;

  /**
   * @brief Evaluate homogeneous transform T \in SE(3) at normalized path parameter s \in [0, 1].
   */
  [[nodiscard]] virtual Eigen::Matrix4d evaluate(double s) const = 0;

  /**
   * @brief Discretize continuous geometric path into N Cartesian poses.
   * @param num_samples Number of discrete waypoints (>= 2)
   */
  [[nodiscard]] virtual std::vector<Eigen::Matrix4d> sample(size_t num_samples) const
  {
    if (num_samples < 2) {
      num_samples = 2;
    }
    std::vector<Eigen::Matrix4d> waypoints;
    waypoints.reserve(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
      double s = static_cast<double>(i) / static_cast<double>(num_samples - 1);
      waypoints.push_back(evaluate(s));
    }
    return waypoints;
  }
};

// ===========================================================================
// CONCRETE STRATEGY 1: LINEAR CARTESIAN PATH (MoveL)
// ===========================================================================

/**
 * @brief Point-to-point straight line in Cartesian space with fixed or interpolated orientation.
 * p(s) = p_start + s * (p_target - p_start)
 */
class CartesianLinePath : public ICartesianPath
{
public:
  CartesianLinePath(
    Eigen::Vector3d start_pos,
    Eigen::Vector3d target_pos,
    Eigen::Matrix3d orientation)
  : start_pos_(std::move(start_pos)),
    target_pos_(std::move(target_pos)),
    orientation_(std::move(orientation))
  {
  }

  [[nodiscard]] std::string getName() const noexcept override
  {
    return "Linear Cartesian Path (MoveL)";
  }

  [[nodiscard]] Eigen::Matrix4d evaluate(double s) const override
  {
    s = std::clamp(s, 0.0, 1.0);
    Eigen::Vector3d p = start_pos_ + s * (target_pos_ - start_pos_);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation_;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  [[nodiscard]] double getLength() const noexcept
  {
    return (target_pos_ - start_pos_).norm();
  }

  [[nodiscard]] const Eigen::Vector3d & getStartPos() const noexcept { return start_pos_; }
  [[nodiscard]] const Eigen::Vector3d & getTargetPos() const noexcept { return target_pos_; }

private:
  Eigen::Vector3d start_pos_;
  Eigen::Vector3d target_pos_;
  Eigen::Matrix3d orientation_;
};

// ===========================================================================
// CONCRETE STRATEGY 2: CIRCULAR CARTESIAN PATH (MoveC)
// ===========================================================================

/**
 * @brief Circular arc or full circle in 3D Cartesian space.
 * p(theta) = c + R * (cos(theta)*u + sin(theta)*v)
 * where u, v form an orthonormal basis spanning the circle plane.
 */
class CartesianCirclePath : public ICartesianPath
{
public:
  /**
   * @brief Construct circle passing through start_pos around center.
   * @param center 3D coordinates of circle center
   * @param start_pos Starting point on the circle circumference
   * @param plane_normal Normal vector to the plane of the circle
   * @param orientation End-effector orientation matrix (kept constant)
   * @param sweep_angle Total subtended angle in radians (default 2*pi for closed circle)
   */
  CartesianCirclePath(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & start_pos,
    const Eigen::Vector3d & plane_normal,
    Eigen::Matrix3d orientation,
    double sweep_angle = 2.0 * M_PI)
  : center_(center),
    orientation_(std::move(orientation)),
    sweep_angle_(sweep_angle)
  {
    Eigen::Vector3d rad_vec = start_pos - center;
    radius_ = rad_vec.norm();

    if (radius_ < 1e-6) {
      // Degenerate circle: fallback to unit radius
      radius_ = 0.01;
      u_axis_ = Eigen::Vector3d::UnitX();
    } else {
      u_axis_ = rad_vec.normalized();
    }

    Eigen::Vector3d n = plane_normal.normalized();
    v_axis_ = n.cross(u_axis_).normalized();
  }

  /**
   * @brief Construct circle from center, radius, and explicit planar axes.
   */
  CartesianCirclePath(
    Eigen::Vector3d center,
    double radius,
    Eigen::Vector3d u_axis,
    Eigen::Vector3d v_axis,
    Eigen::Matrix3d orientation,
    double sweep_angle = 2.0 * M_PI)
  : center_(std::move(center)),
    radius_(std::abs(radius)),
    u_axis_(u_axis.normalized()),
    v_axis_(v_axis.normalized()),
    orientation_(std::move(orientation)),
    sweep_angle_(sweep_angle)
  {
  }

  [[nodiscard]] std::string getName() const noexcept override
  {
    return "Circular Cartesian Path (MoveC)";
  }

  [[nodiscard]] Eigen::Matrix4d evaluate(double s) const override
  {
    s = std::clamp(s, 0.0, 1.0);
    double theta = s * sweep_angle_;
    Eigen::Vector3d p = center_ + radius_ * (std::cos(theta) * u_axis_ + std::sin(theta) * v_axis_);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation_;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  [[nodiscard]] double getRadius() const noexcept { return radius_; }
  [[nodiscard]] const Eigen::Vector3d & getCenter() const noexcept { return center_; }
  [[nodiscard]] double getArcLength() const noexcept { return radius_ * std::abs(sweep_angle_); }

private:
  Eigen::Vector3d center_;
  double radius_{0.0};
  Eigen::Vector3d u_axis_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d v_axis_{Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d orientation_{Eigen::Matrix3d::Identity()};
  double sweep_angle_{2.0 * M_PI};
};

// ===========================================================================
// CONCRETE STRATEGY 3: RECTANGULAR CARTESIAN PATH (MoveRect)
// ===========================================================================

/**
 * @brief Closed rectangular planar loop with 4 linear segments.
 * Corners: C0 (start) -> C1 -> C2 -> C3 -> C0 (return to start).
 */
class CartesianRectanglePath : public ICartesianPath
{
public:
  CartesianRectanglePath(
    Eigen::Vector3d origin_corner,
    double width,
    double height,
    Eigen::Vector3d width_dir,
    Eigen::Vector3d height_dir,
    Eigen::Matrix3d orientation)
  : orientation_(std::move(orientation))
  {
    Eigen::Vector3d u = width_dir.normalized();
    Eigen::Vector3d v = height_dir.normalized();

    corners_.reserve(5);
    corners_.push_back(origin_corner);                           // C0: origin
    corners_.push_back(origin_corner + width * u);               // C1: +width
    corners_.push_back(origin_corner + width * u + height * v);  // C2: +width +height
    corners_.push_back(origin_corner + height * v);              // C3: +height
    corners_.push_back(origin_corner);                           // C4 = C0: closed loop
  }

  [[nodiscard]] std::string getName() const noexcept override
  {
    return "Rectangular Cartesian Path (MoveRect)";
  }

  [[nodiscard]] Eigen::Matrix4d evaluate(double s) const override
  {
    s = std::clamp(s, 0.0, 1.0);
    // 4 segments of equal normalized length 0.25
    double seg_param = s * 4.0;
    size_t seg_idx = static_cast<size_t>(std::floor(seg_param));
    if (seg_idx >= 4) {
      seg_idx = 3;
      seg_param = 4.0;
    }
    double local_s = seg_param - static_cast<double>(seg_idx);

    Eigen::Vector3d p0 = corners_[seg_idx];
    Eigen::Vector3d p1 = corners_[seg_idx + 1];
    Eigen::Vector3d p = p0 + local_s * (p1 - p0);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation_;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  [[nodiscard]] const std::vector<Eigen::Vector3d> & getCorners() const noexcept
  {
    return corners_;
  }

private:
  std::vector<Eigen::Vector3d> corners_;
  Eigen::Matrix3d orientation_;
};

// ===========================================================================
// CONCRETE STRATEGY 4: 3D SPIRAL CARTESIAN PATH (MoveSpiral)
// ===========================================================================

/**
 * @brief 3D Conical / Archimedean Spiral in Cartesian Space.
 * Radius and axial height vary linearly with path parameter s:
 * r(s) = r_start + s * (r_end - r_start)
 * theta(s) = s * 2 * pi * num_turns
 * p(s) = center + r(s) * (cos(theta)*u + sin(theta)*v) + s * axial_height * w
 */
class CartesianSpiralPath : public ICartesianPath
{
public:
  CartesianSpiralPath(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & start_pos,
    double end_radius,
    double axial_height,
    double num_turns,
    Eigen::Matrix3d orientation,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0))
  : center_(center),
    end_radius_(end_radius),
    axial_height_(axial_height),
    num_turns_(num_turns),
    orientation_(std::move(orientation))
  {
    w_axis_ = axial_dir.normalized();
    Eigen::Vector3d rad_vec = (start_pos - center) - ((start_pos - center).dot(w_axis_)) * w_axis_;
    start_radius_ = rad_vec.norm();
    if (start_radius_ < 1e-6) {
      start_radius_ = 0.02;
      u_axis_ = Eigen::Vector3d::UnitX();
    } else {
      u_axis_ = rad_vec.normalized();
    }
    v_axis_ = w_axis_.cross(u_axis_).normalized();
  }

  [[nodiscard]] std::string getName() const noexcept override
  {
    return "3D Spiral Cartesian Path (MoveSpiral)";
  }

  [[nodiscard]] Eigen::Matrix4d evaluate(double s) const override
  {
    s = std::clamp(s, 0.0, 1.0);
    double theta = s * (2.0 * M_PI * num_turns_);
    double r = start_radius_ + s * (end_radius_ - start_radius_);

    Eigen::Vector3d p = center_ +
      r * (std::cos(theta) * u_axis_ + std::sin(theta) * v_axis_) +
      (s * axial_height_) * w_axis_;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation_;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  [[nodiscard]] double getStartRadius() const noexcept { return start_radius_; }
  [[nodiscard]] double getEndRadius() const noexcept { return end_radius_; }
  [[nodiscard]] double getAxialHeight() const noexcept { return axial_height_; }
  [[nodiscard]] double getNumTurns() const noexcept { return num_turns_; }

private:
  Eigen::Vector3d center_;
  double start_radius_{0.0};
  double end_radius_{0.0};
  double axial_height_{0.0};
  double num_turns_{2.0};
  Eigen::Vector3d u_axis_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d v_axis_{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d w_axis_{Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d orientation_{Eigen::Matrix3d::Identity()};
};

// ===========================================================================
// CONCRETE STRATEGY 5: 3D CYLINDRICAL HELIX CARTESIAN PATH (MoveHelix)
// ===========================================================================

/**
 * @brief 3D Cylindrical Helix in Cartesian Space.
 * Constant radius R with linear axial progression:
 * theta(s) = s * 2 * pi * num_turns
 * p(s) = center + R * (cos(theta)*u + sin(theta)*v) + s * total_height * w
 */
class CartesianHelixPath : public ICartesianPath
{
public:
  CartesianHelixPath(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & start_pos,
    double total_height,
    double num_turns,
    Eigen::Matrix3d orientation,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0))
  : center_(center),
    total_height_(total_height),
    num_turns_(num_turns),
    orientation_(std::move(orientation))
  {
    w_axis_ = axial_dir.normalized();
    Eigen::Vector3d rad_vec = (start_pos - center) - ((start_pos - center).dot(w_axis_)) * w_axis_;
    radius_ = rad_vec.norm();
    if (radius_ < 1e-6) {
      radius_ = 0.05;
      u_axis_ = Eigen::Vector3d::UnitX();
    } else {
      u_axis_ = rad_vec.normalized();
    }
    v_axis_ = w_axis_.cross(u_axis_).normalized();
  }

  [[nodiscard]] std::string getName() const noexcept override
  {
    return "3D Helical Cartesian Path (MoveHelix)";
  }

  [[nodiscard]] Eigen::Matrix4d evaluate(double s) const override
  {
    s = std::clamp(s, 0.0, 1.0);
    double theta = s * (2.0 * M_PI * num_turns_);

    Eigen::Vector3d p = center_ +
      radius_ * (std::cos(theta) * u_axis_ + std::sin(theta) * v_axis_) +
      (s * total_height_) * w_axis_;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation_;
    T.block<3, 1>(0, 3) = p;
    return T;
  }

  [[nodiscard]] double getRadius() const noexcept { return radius_; }
  [[nodiscard]] double getTotalHeight() const noexcept { return total_height_; }
  [[nodiscard]] double getPitch() const noexcept { return num_turns_ > 1e-6 ? total_height_ / num_turns_ : 0.0; }
  [[nodiscard]] double getNumTurns() const noexcept { return num_turns_; }

private:
  Eigen::Vector3d center_;
  double radius_{0.0};
  double total_height_{0.0};
  double num_turns_{2.0};
  Eigen::Vector3d u_axis_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d v_axis_{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d w_axis_{Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d orientation_{Eigen::Matrix3d::Identity()};
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__CARTESIAN_PATH_HPP_

