#ifndef DOBOT_TRAJECTORY__TRAJECTORY_PLANNING_ENGINE_HPP_
#define DOBOT_TRAJECTORY__TRAJECTORY_PLANNING_ENGINE_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace dobot_trajectory
{

// ===========================================================================
// 1. DATA MODELS & TELEMETRY (SINGLE RESPONSIBILITY PRINCIPLE)
// ===========================================================================

/// Strongly typed strategy selection identifiers
enum class InterpolatorType
{
  CUBIC_BSPLINE,
  LINEAR_SEGMENT
};

enum class ProfilerType
{
  TRAPEZOIDAL,
  QUINTIC_POLYNOMIAL
};

/// 1D Geometric evaluation along S(u) \in R^n
struct PathEvaluation
{
  Eigen::VectorXd position;      ///< S(u) \in R^n
  Eigen::VectorXd velocity_du;   ///< dS/du (first derivative w.r.t path parameter u)
  Eigen::VectorXd accel_du2;     ///< d^2S/du^2 (second derivative w.r.t path parameter u)
};

/// 1D Timing profile state along u(t)
struct ProfileState
{
  double u{0.0};       ///< Path parameter u \in [0, 1]
  double u_dot{0.0};   ///< du/dt (1/s)
  double u_ddot{0.0};  ///< d^2u/dt^2 (1/s^2)
};

/// Continuous Joint Space Trajectory State at timestamp t
struct JointTrajectoryState
{
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  double time_from_start{0.0};
};

/// Multi-axis Manipulator Physical Limits Encapsulation
struct KinematicLimits
{
  Eigen::VectorXd max_velocities;      ///< Maximum joint velocities (rad/s)
  Eigen::VectorXd max_accelerations;   ///< Maximum joint accelerations (rad/s^2)

  [[nodiscard]] bool isValid(size_t expected_dof) const noexcept
  {
    return static_cast<size_t>(max_velocities.size()) == expected_dof &&
           static_cast<size_t>(max_accelerations.size()) == expected_dof &&
           (max_velocities.array() > 0.0).all() &&
           (max_accelerations.array() > 0.0).all();
  }
};

/// Diagnostic parameters for Trapezoidal Velocity Profile
struct TrapezoidalProfileParams
{
  double t_accel{0.0};      ///< Blending / acceleration time (s)
  double t_cruise{0.0};     ///< Constant velocity time (s)
  double t_decel{0.0};      ///< Deceleration time (s)
  double total_time{0.0};   ///< Total duration T = t_accel + t_cruise + t_decel (s)
  double cruise_vel{0.0};   ///< Peak reachable velocity in u-space (1/s)
  double accel{0.0};        ///< Effective acceleration limit in u-space (1/s^2)
  bool is_triangular{false};///< True if distance is too short to reach nominal cruise vel
};

/// Kinematic utilization and telemetry report
struct TrajectoryUtilizationReport
{
  Eigen::VectorXd peak_velocities;       ///< Peak observed joint velocities (rad/s)
  Eigen::VectorXd peak_accelerations;    ///< Peak observed joint accelerations (rad/s^2)
  Eigen::VectorXd velocity_utilization;  ///< Ratio of peak velocity to limit [0, 1]
  Eigen::VectorXd accel_utilization;     ///< Ratio of peak acceleration to limit [0, 1]
  bool limits_satisfied{true};
};

/// Unified Trajectory Result Object (Value Object Pattern)
struct TrajectoryResult
{
  bool success{false};
  std::string status_message;
  double duration{0.0};
  size_t num_points{0};
  trajectory_msgs::msg::JointTrajectory trajectory;
  TrajectoryUtilizationReport report;
};

// ===========================================================================
// 2. STRATEGY PATTERN: GEOMETRIC PATH INTERPOLATION INTERFACE (ISP & DIP)
// ===========================================================================

class IPathInterpolationStrategy
{
public:
  virtual ~IPathInterpolationStrategy() = default;

  [[nodiscard]] virtual std::string getName() const noexcept = 0;

  /**
   * @brief Fit continuous geometric curve S(u) through discrete joint waypoints.
   * @param waypoints Sequence of joint configurations (m >= 1)
   */
  virtual void fit(const std::vector<Eigen::VectorXd> & waypoints) = 0;

  /**
   * @brief Evaluate position and analytical derivatives at path parameter u \in [0, 1].
   */
  [[nodiscard]] virtual PathEvaluation evaluate(double u) const = 0;

  /**
   * @brief Get dimension (degrees of freedom) of the path.
   */
  [[nodiscard]] virtual size_t getDOF() const noexcept = 0;
};

// ===========================================================================
// CONCRETE STRATEGY 1: CUBIC B-SPLINE INTERPOLATOR
// ===========================================================================

/**
 * @brief C^2 continuous cubic spline interpolation passing exactly through waypoints.
 * Solves a tridiagonal system for analytical C^2 continuity with zero endpoint curvature.
 */
class CubicBSplineInterpolator : public IPathInterpolationStrategy
{
public:
  [[nodiscard]] std::string getName() const noexcept override { return "C^2 Cubic B-Spline Interpolator"; }

  void fit(const std::vector<Eigen::VectorXd> & waypoints) override;
  [[nodiscard]] PathEvaluation evaluate(double u) const override;
  [[nodiscard]] size_t getDOF() const noexcept override { return dof_; }

private:
  size_t dof_{6};
  size_t num_segments_{0};
  std::vector<double> u_knots_;

  std::vector<Eigen::VectorXd> a_;
  std::vector<Eigen::VectorXd> b_;
  std::vector<Eigen::VectorXd> c_;
  std::vector<Eigen::VectorXd> d_;
};

// ===========================================================================
// CONCRETE STRATEGY 2: LINEAR SEGMENT INTERPOLATOR
// ===========================================================================

/**
 * @brief Piecewise linear joint interpolation with exact waypoint pass-through.
 * Provides C^0 continuity with piecewise-constant derivatives.
 */
class LinearSegmentInterpolator : public IPathInterpolationStrategy
{
public:
  [[nodiscard]] std::string getName() const noexcept override { return "Linear Segment Interpolator (C^0)"; }

  void fit(const std::vector<Eigen::VectorXd> & waypoints) override;
  [[nodiscard]] PathEvaluation evaluate(double u) const override;
  [[nodiscard]] size_t getDOF() const noexcept override { return dof_; }

private:
  size_t dof_{6};
  size_t num_segments_{0};
  std::vector<double> u_knots_;
  std::vector<Eigen::VectorXd> waypoints_;
};

// ===========================================================================
// 3. STRATEGY PATTERN: TIME PARAMETERIZATION INTERFACE (ISP & DIP)
// ===========================================================================

class ITimeParameterizationStrategy
{
public:
  virtual ~ITimeParameterizationStrategy() = default;

  [[nodiscard]] virtual std::string getName() const noexcept = 0;

  /**
   * @brief Compute path timing u(t) satisfying robot velocity and acceleration limits.
   * @param path Continuous geometric path S(u)
   * @param limits Predefined joint velocity and acceleration limits
   * @return Total trajectory execution duration in seconds
   */
  virtual double computeProfile(
    const IPathInterpolationStrategy & path,
    const KinematicLimits & limits) = 0;

  double computeProfile(
    const IPathInterpolationStrategy & path,
    const Eigen::VectorXd & max_vel,
    const Eigen::VectorXd & max_acc);

  /**
   * @brief Evaluate u(t), du/dt, and d^2u/dt^2 at timestamp t \in [0, T].
   */
  [[nodiscard]] virtual ProfileState evaluate(double t) const = 0;

  /**
   * @brief Get total execution duration T (s).
   */
  [[nodiscard]] virtual double getDuration() const noexcept = 0;
};

// ===========================================================================
// CONCRETE STRATEGY 3: TRAPEZOIDAL VELOCITY PROFILER
// ===========================================================================

/**
 * @brief Classical bang-coast-bang trapezoidal time parameterization.
 * Evaluates blending time, cruise velocity, and triangular profile fallback.
 */
class TrapezoidalVelocityProfiler : public ITimeParameterizationStrategy
{
public:
  [[nodiscard]] std::string getName() const noexcept override
  {
    return "Trapezoidal Velocity Profiler (Bang-Coast-Bang)";
  }

  double computeProfile(
    const IPathInterpolationStrategy & path,
    const KinematicLimits & limits) override;

  [[nodiscard]] ProfileState evaluate(double t) const override;
  [[nodiscard]] double getDuration() const noexcept override { return params_.total_time; }

  [[nodiscard]] const TrapezoidalProfileParams & getParams() const noexcept { return params_; }

private:
  TrapezoidalProfileParams params_;
};

// ===========================================================================
// CONCRETE STRATEGY 4: QUINTIC POLYNOMIAL (S-CURVE) PROFILER
// ===========================================================================

/**
 * @brief Jerk-bounded minimum-jerk polynomial parameterization:
 * u(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5 (Zero boundary acceleration and jerk).
 */
class QuinticPolynomialProfiler : public ITimeParameterizationStrategy
{
public:
  [[nodiscard]] std::string getName() const noexcept override
  {
    return "Quintic Polynomial Profiler (Jerk-Bounded S-Curve)";
  }

  double computeProfile(
    const IPathInterpolationStrategy & path,
    const KinematicLimits & limits) override;

  [[nodiscard]] ProfileState evaluate(double t) const override;
  [[nodiscard]] double getDuration() const noexcept override { return total_time_; }

private:
  double total_time_{0.0};
};

// ===========================================================================
// 4. FACTORY PATTERN: STRATEGY CREATION (OPEN/CLOSED PRINCIPLE)
// ===========================================================================

class TrajectoryStrategyFactory
{
public:
  [[nodiscard]] static std::shared_ptr<IPathInterpolationStrategy> createInterpolator(
    InterpolatorType type);

  [[nodiscard]] static std::shared_ptr<ITimeParameterizationStrategy> createProfiler(
    ProfilerType type);
};

// ===========================================================================
// 5. UNIFIED MASTER ENGINE: TrajectoryPlanningEngine (SRP, DIP, BUILDER)
// ===========================================================================

class TrajectoryPlanningEngine
{
public:
  /**
   * @brief Construct with Dobot 6-axis hardware limits and joint naming
   */
  explicit TrajectoryPlanningEngine(
    const std::vector<std::string> & joint_names = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
    const Eigen::VectorXd & max_velocities = (Eigen::VectorXd(6) << 3.14, 3.14, 3.14, 3.14, 3.14, 3.14).finished(),
    const Eigen::VectorXd & max_accelerations = (Eigen::VectorXd(6) << 10.0, 10.0, 10.0, 15.0, 15.0, 15.0).finished());

  // -------------------------------------------------------------------------
  // Strategy Injection & Builder Configuration (Fluent Interface)
  // -------------------------------------------------------------------------
  TrajectoryPlanningEngine & setInterpolator(InterpolatorType type);
  TrajectoryPlanningEngine & setInterpolationStrategy(
    std::shared_ptr<IPathInterpolationStrategy> strategy) noexcept;
  [[nodiscard]] const IPathInterpolationStrategy & getInterpolationStrategy() const noexcept
  {
    return *interpolation_strategy_;
  }

  TrajectoryPlanningEngine & setProfiler(ProfilerType type);
  TrajectoryPlanningEngine & setTimeParameterizationStrategy(
    std::shared_ptr<ITimeParameterizationStrategy> strategy) noexcept;
  [[nodiscard]] const ITimeParameterizationStrategy & getTimeParameterizationStrategy() const noexcept
  {
    return *time_strategy_;
  }

  TrajectoryPlanningEngine & setJointLimits(
    const Eigen::VectorXd & max_vel,
    const Eigen::VectorXd & max_acc);

  TrajectoryPlanningEngine & setKinematicLimits(KinematicLimits limits);
  TrajectoryPlanningEngine & setJointNames(std::vector<std::string> names);

  [[nodiscard]] const KinematicLimits & getKinematicLimits() const noexcept { return limits_; }
  [[nodiscard]] const Eigen::VectorXd & getMaxVelocities() const noexcept { return limits_.max_velocities; }
  [[nodiscard]] const Eigen::VectorXd & getMaxAccelerations() const noexcept { return limits_.max_accelerations; }
  [[nodiscard]] const std::vector<std::string> & getJointNames() const noexcept { return joint_names_; }

  // -------------------------------------------------------------------------
  // Core Trajectory Planning API
  // -------------------------------------------------------------------------

  /**
   * @brief Transform discrete joint waypoints into a complete TrajectoryResult with telemetry.
   * @param waypoints Ordered joint configurations from MoveIt / RRT planner
   * @param sample_dt Time step resolution (default: 0.02s for 50Hz)
   * @param frame_id Frame identifier (default: "base_link")
   * @return TrajectoryResult complete with status, duration, message, and telemetry
   */
  [[nodiscard]] TrajectoryResult plan(
    const std::vector<Eigen::VectorXd> & waypoints,
    double sample_dt = 0.02,
    const std::string & frame_id = "base_link") const;

  /**
   * @brief Backward-compatible convenience wrapper returning JointTrajectory message.
   */
  [[nodiscard]] trajectory_msgs::msg::JointTrajectory parameterize(
    const std::vector<Eigen::VectorXd> & waypoints,
    double sample_dt = 0.02,
    const std::string & frame_id = "base_link") const;

  /**
   * @brief Continuous real-time zero-allocation state evaluation at timestamp t.
   * Suitable for high-frequency (e.g. 1 kHz) ros2_control hardware execution loops.
   */
  [[nodiscard]] JointTrajectoryState sampleAt(double t) const;

  /**
   * @brief Helper: Compute trajectory duration without generating all samples
   */
  [[nodiscard]] double computeDuration(const std::vector<Eigen::VectorXd> & waypoints) const;

  /**
   * @brief Analyze kinematic limit compliance and report peak utilization ratios.
   */
  [[nodiscard]] TrajectoryUtilizationReport evaluateUtilization(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;

private:
  std::vector<std::string> joint_names_;
  KinematicLimits limits_;

  std::shared_ptr<IPathInterpolationStrategy> interpolation_strategy_;
  std::shared_ptr<ITimeParameterizationStrategy> time_strategy_;
};

}  // namespace dobot_trajectory

#endif  // DOBOT_TRAJECTORY__TRAJECTORY_PLANNING_ENGINE_HPP_
