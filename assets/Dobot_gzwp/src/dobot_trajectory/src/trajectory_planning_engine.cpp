#include "dobot_trajectory/trajectory_planning_engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dobot_trajectory
{

// ===========================================================================
// 1. CUBIC B-SPLINE INTERPOLATOR IMPLEMENTATION
// ===========================================================================

void CubicBSplineInterpolator::fit(const std::vector<Eigen::VectorXd> & waypoints)
{
  if (waypoints.empty()) {
    throw std::invalid_argument("CubicBSplineInterpolator::fit: Waypoints list is empty.");
  }

  dof_ = waypoints[0].size();
  for (size_t i = 1; i < waypoints.size(); ++i) {
    if (static_cast<size_t>(waypoints[i].size()) != dof_) {
      throw std::invalid_argument(
        "CubicBSplineInterpolator::fit: Inconsistent waypoint DOF dimensions.");
    }
  }

  // Handle single-waypoint edge case (stationary point)
  if (waypoints.size() == 1) {
    num_segments_ = 1;
    u_knots_ = {0.0, 1.0};
    a_ = {waypoints[0]};
    b_ = {Eigen::VectorXd::Zero(dof_)};
    c_ = {Eigen::VectorXd::Zero(dof_)};
    d_ = {Eigen::VectorXd::Zero(dof_)};
    return;
  }

  const size_t m = waypoints.size() - 1;  // Number of segments
  num_segments_ = m;

  // 1. Chord-length parameterization: compute cumulative distance along path
  std::vector<double> seg_lengths(m, 0.0);
  double total_length = 0.0;
  for (size_t k = 0; k < m; ++k) {
    seg_lengths[k] = (waypoints[k + 1] - waypoints[k]).norm();
    total_length += seg_lengths[k];
  }

  u_knots_.resize(m + 1, 0.0);
  u_knots_[0] = 0.0;
  if (total_length < 1e-9) {
    // All waypoints coincide: distribute knots uniformly
    for (size_t k = 1; k <= m; ++k) {
      u_knots_[k] = static_cast<double>(k) / static_cast<double>(m);
    }
  } else {
    // Cumulative normalized chord length with minimum segment length guard
    double current_dist = 0.0;
    for (size_t k = 0; k < m - 1; ++k) {
      current_dist += seg_lengths[k];
      u_knots_[k + 1] = current_dist / total_length;
      if (u_knots_[k + 1] <= u_knots_[k]) {
        u_knots_[k + 1] = u_knots_[k] + 1e-6;
      }
    }
    u_knots_[m] = 1.0;
  }

  // Step lengths h_k = u_{k+1} - u_k
  std::vector<double> h(m, 0.0);
  for (size_t k = 0; k < m; ++k) {
    h[k] = u_knots_[k + 1] - u_knots_[k];
    if (h[k] <= 1e-12) {
      h[k] = 1e-6;  // Numerical safety guard
    }
  }

  // Handle two-waypoint linear segment
  if (m == 1) {
    a_ = {waypoints[0]};
    b_ = {(waypoints[1] - waypoints[0]) / h[0]};
    c_ = {Eigen::VectorXd::Zero(dof_)};
    d_ = {Eigen::VectorXd::Zero(dof_)};
    return;
  }

  // 2. Setup Tridiagonal System for Natural Spline (M_0 = M_m = 0)
  // Interior knots k = 1, ..., m-1 (N = m - 1 equations)
  const size_t N = m - 1;
  std::vector<double> alpha(N, 0.0);  // Main diagonal
  std::vector<double> beta(N, 0.0);   // Sub diagonal (lower)
  std::vector<double> gamma(N, 0.0);  // Super diagonal (upper)
  std::vector<Eigen::VectorXd> R(N, Eigen::VectorXd::Zero(dof_));  // RHS vectors

  for (size_t i = 0; i < N; ++i) {
    const size_t k = i + 1;
    alpha[i] = 2.0 * (h[k - 1] + h[k]);
    if (i > 0) {
      beta[i] = h[k - 1];
    }
    if (i < N - 1) {
      gamma[i] = h[k];
    }
    const Eigen::VectorXd delta_next = (waypoints[k + 1] - waypoints[k]) / h[k];
    const Eigen::VectorXd delta_prev = (waypoints[k] - waypoints[k - 1]) / h[k - 1];
    R[i] = 6.0 * (delta_next - delta_prev);
  }

  // 3. Solve Tridiagonal System via Thomas Algorithm (TDMA, O(m) time complexity)
  std::vector<double> c_prime(N, 0.0);
  std::vector<Eigen::VectorXd> d_prime(N, Eigen::VectorXd::Zero(dof_));

  c_prime[0] = gamma[0] / alpha[0];
  d_prime[0] = R[0] / alpha[0];

  for (size_t i = 1; i < N; ++i) {
    const double denom = alpha[i] - beta[i] * c_prime[i - 1];
    if (i < N - 1) {
      c_prime[i] = gamma[i] / denom;
    }
    d_prime[i] = (R[i] - beta[i] * d_prime[i - 1]) / denom;
  }

  // Back substitution to recover second derivatives M_k = S''(u_k)
  std::vector<Eigen::VectorXd> M(m + 1, Eigen::VectorXd::Zero(dof_));
  M[0] = Eigen::VectorXd::Zero(dof_);
  M[m] = Eigen::VectorXd::Zero(dof_);

  M[N] = d_prime[N - 1];
  for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
    M[i + 1] = d_prime[i] - c_prime[i] * M[i + 2];
  }

  // 4. Compute Cubic Polynomial Coefficients for each segment k in [0, m-1]
  a_.resize(m);
  b_.resize(m);
  c_.resize(m);
  d_.resize(m);

  for (size_t k = 0; k < m; ++k) {
    a_[k] = waypoints[k];
    b_[k] = (waypoints[k + 1] - waypoints[k]) / h[k] -
            (h[k] / 6.0) * (2.0 * M[k] + M[k + 1]);
    c_[k] = 0.5 * M[k];
    d_[k] = (M[k + 1] - M[k]) / (6.0 * h[k]);
  }
}

PathEvaluation CubicBSplineInterpolator::evaluate(double u) const
{
  if (num_segments_ == 0 || a_.empty()) {
    throw std::runtime_error("CubicBSplineInterpolator::evaluate: Interpolator has not been fitted.");
  }

  const double clamped_u = std::clamp(u, 0.0, 1.0);

  size_t k = 0;
  if (clamped_u <= u_knots_[0]) {
    k = 0;
  } else if (clamped_u >= u_knots_.back()) {
    k = num_segments_ - 1;
  } else {
    auto it = std::upper_bound(u_knots_.begin(), u_knots_.end(), clamped_u);
    k = static_cast<size_t>(std::distance(u_knots_.begin(), it) - 1);
    if (k >= num_segments_) {
      k = num_segments_ - 1;
    }
  }

  const double du = clamped_u - u_knots_[k];
  const double du2 = du * du;
  const double du3 = du2 * du;

  PathEvaluation eval;
  eval.position = a_[k] + b_[k] * du + c_[k] * du2 + d_[k] * du3;
  eval.velocity_du = b_[k] + 2.0 * c_[k] * du + 3.0 * d_[k] * du2;
  eval.accel_du2 = 2.0 * c_[k] + 6.0 * d_[k] * du;

  return eval;
}

// ===========================================================================
// 2. LINEAR SEGMENT INTERPOLATOR IMPLEMENTATION
// ===========================================================================

void LinearSegmentInterpolator::fit(const std::vector<Eigen::VectorXd> & waypoints)
{
  if (waypoints.empty()) {
    throw std::invalid_argument("LinearSegmentInterpolator::fit: Waypoints list is empty.");
  }

  dof_ = waypoints[0].size();
  waypoints_ = waypoints;

  if (waypoints.size() == 1) {
    num_segments_ = 1;
    u_knots_ = {0.0, 1.0};
    return;
  }

  const size_t m = waypoints.size() - 1;
  num_segments_ = m;

  double total_length = 0.0;
  std::vector<double> seg_lengths(m, 0.0);
  for (size_t k = 0; k < m; ++k) {
    seg_lengths[k] = (waypoints[k + 1] - waypoints[k]).norm();
    total_length += seg_lengths[k];
  }

  u_knots_.resize(m + 1, 0.0);
  u_knots_[0] = 0.0;
  if (total_length < 1e-9) {
    for (size_t k = 1; k <= m; ++k) {
      u_knots_[k] = static_cast<double>(k) / static_cast<double>(m);
    }
  } else {
    double current = 0.0;
    for (size_t k = 0; k < m - 1; ++k) {
      current += seg_lengths[k];
      u_knots_[k + 1] = current / total_length;
      if (u_knots_[k + 1] <= u_knots_[k]) {
        u_knots_[k + 1] = u_knots_[k] + 1e-6;
      }
    }
    u_knots_[m] = 1.0;
  }
}

PathEvaluation LinearSegmentInterpolator::evaluate(double u) const
{
  if (waypoints_.empty()) {
    throw std::runtime_error("LinearSegmentInterpolator::evaluate: Not fitted.");
  }
  if (waypoints_.size() == 1) {
    return {waypoints_[0], Eigen::VectorXd::Zero(dof_), Eigen::VectorXd::Zero(dof_)};
  }

  const double clamped_u = std::clamp(u, 0.0, 1.0);
  size_t k = 0;
  if (clamped_u <= u_knots_[0]) {
    k = 0;
  } else if (clamped_u >= u_knots_.back()) {
    k = num_segments_ - 1;
  } else {
    auto it = std::upper_bound(u_knots_.begin(), u_knots_.end(), clamped_u);
    k = static_cast<size_t>(std::distance(u_knots_.begin(), it) - 1);
    if (k >= num_segments_) {
      k = num_segments_ - 1;
    }
  }

  const double h_k = std::max(u_knots_[k + 1] - u_knots_[k], 1e-9);
  const double tau = std::clamp((clamped_u - u_knots_[k]) / h_k, 0.0, 1.0);

  PathEvaluation eval;
  eval.position = waypoints_[k] + tau * (waypoints_[k + 1] - waypoints_[k]);
  eval.velocity_du = (waypoints_[k + 1] - waypoints_[k]) / h_k;
  eval.accel_du2 = Eigen::VectorXd::Zero(dof_);
  return eval;
}

// ===========================================================================
// 3. TIME PARAMETERIZATION BASE WRAPPER
// ===========================================================================

double ITimeParameterizationStrategy::computeProfile(
  const IPathInterpolationStrategy & path,
  const Eigen::VectorXd & max_vel,
  const Eigen::VectorXd & max_acc)
{
  KinematicLimits limits{max_vel, max_acc};
  return computeProfile(path, limits);
}

// ===========================================================================
// 4. TRAPEZOIDAL VELOCITY PROFILER IMPLEMENTATION
// ===========================================================================

double TrapezoidalVelocityProfiler::computeProfile(
  const IPathInterpolationStrategy & path,
  const KinematicLimits & limits)
{
  const size_t dof = path.getDOF();
  if (!limits.isValid(dof)) {
    throw std::invalid_argument(
      "TrapezoidalVelocityProfiler::computeProfile: Invalid kinematic limits dimensions or values.");
  }

  // Sample S'(u) and S''(u) along u in [0, 1] to identify peak geometric scaling
  constexpr size_t NUM_SAMPLES = 101;
  Eigen::VectorXd peak_dS_du = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd peak_d2S_du2 = Eigen::VectorXd::Zero(dof);

  for (size_t s = 0; s < NUM_SAMPLES; ++s) {
    const double u_sample = static_cast<double>(s) / static_cast<double>(NUM_SAMPLES - 1);
    const PathEvaluation eval = path.evaluate(u_sample);
    for (size_t i = 0; i < dof; ++i) {
      peak_dS_du[i] = std::max(peak_dS_du[i], std::abs(eval.velocity_du[i]));
      peak_d2S_du2[i] = std::max(peak_d2S_du2[i], std::abs(eval.accel_du2[i]));
    }
  }

  // Acceleration budget partitioning:
  // |q_ddot_i| <= |S''_i|*(u_dot)^2 + |S'_i|*|u_ddot| <= A_max,i
  // We allocate 60% of A_max to tangential acceleration and 40% to path curvature.
  double u_dot_limit = 1e6;
  double u_ddot_limit = 1e6;

  for (size_t i = 0; i < dof; ++i) {
    const double ds = std::max(peak_dS_du[i], 1e-4);
    const double v_limit = limits.max_velocities[i] / ds;
    const double a_limit = (0.60 * limits.max_accelerations[i]) / ds;
    u_dot_limit = std::min(u_dot_limit, v_limit);
    u_ddot_limit = std::min(u_ddot_limit, a_limit);

    if (peak_d2S_du2[i] > 1e-4) {
      const double v_curv_limit = std::sqrt((0.40 * limits.max_accelerations[i]) / peak_d2S_du2[i]);
      u_dot_limit = std::min(u_dot_limit, v_curv_limit);
    }
  }

  u_dot_limit = std::max(u_dot_limit, 1e-3);
  u_ddot_limit = std::max(u_ddot_limit, 1e-3);

  const double delta_u = 1.0;
  const double d_accel_nominal = (u_dot_limit * u_dot_limit) / (2.0 * u_ddot_limit);

  params_.accel = u_ddot_limit;

  if (delta_u > 2.0 * d_accel_nominal) {
    params_.is_triangular = false;
    params_.cruise_vel = u_dot_limit;
    params_.t_accel = params_.cruise_vel / params_.accel;
    params_.t_decel = params_.t_accel;
    const double d_cruise = delta_u - 2.0 * d_accel_nominal;
    params_.t_cruise = d_cruise / params_.cruise_vel;
    params_.total_time = params_.t_accel + params_.t_cruise + params_.t_decel;
  } else {
    params_.is_triangular = true;
    params_.cruise_vel = std::sqrt(params_.accel * delta_u);
    params_.t_accel = params_.cruise_vel / params_.accel;
    params_.t_cruise = 0.0;
    params_.t_decel = params_.t_accel;
    params_.total_time = 2.0 * params_.t_accel;
  }

  // Closed-loop fine-grid verification and analytical time dilation
  constexpr size_t VERIFY_SAMPLES = 200;
  double max_vel_ratio = 0.0;
  double max_acc_ratio = 0.0;

  for (size_t s = 0; s <= VERIFY_SAMPLES; ++s) {
    const double t_test = params_.total_time * (static_cast<double>(s) / static_cast<double>(VERIFY_SAMPLES));
    const ProfileState st = evaluate(t_test);
    const PathEvaluation geom = path.evaluate(st.u);

    for (size_t i = 0; i < dof; ++i) {
      const double v_joint = std::abs(geom.velocity_du[i] * st.u_dot);
      const double a_joint = std::abs(
        geom.accel_du2[i] * (st.u_dot * st.u_dot) +
        geom.velocity_du[i] * st.u_ddot);

      max_vel_ratio = std::max(max_vel_ratio, v_joint / limits.max_velocities[i]);
      max_acc_ratio = std::max(max_acc_ratio, a_joint / limits.max_accelerations[i]);
    }
  }

  const double time_dilation = std::max({1.0, max_vel_ratio, std::sqrt(max_acc_ratio)}) * 1.005;

  if (time_dilation > 1.0) {
    params_.t_accel *= time_dilation;
    params_.t_cruise *= time_dilation;
    params_.t_decel *= time_dilation;
    params_.total_time *= time_dilation;
    params_.cruise_vel /= time_dilation;
    params_.accel /= (time_dilation * time_dilation);
  }

  return params_.total_time;
}

ProfileState TrapezoidalVelocityProfiler::evaluate(double t) const
{
  ProfileState state;
  if (t <= 0.0) {
    state.u = 0.0;
    state.u_dot = 0.0;
    state.u_ddot = 0.0;
    return state;
  }

  if (t >= params_.total_time) {
    state.u = 1.0;
    state.u_dot = 0.0;
    state.u_ddot = 0.0;
    return state;
  }

  const double t1 = params_.t_accel;
  const double t2 = params_.t_accel + params_.t_cruise;

  if (t < t1) {
    // Phase 1: Acceleration (Blending time t_a)
    state.u_ddot = params_.accel;
    state.u_dot = params_.accel * t;
    state.u = 0.5 * params_.accel * t * t;
  } else if (t < t2) {
    // Phase 2: Constant Cruise Velocity
    state.u_ddot = 0.0;
    state.u_dot = params_.cruise_vel;
    const double u_at_t1 = 0.5 * params_.accel * t1 * t1;
    state.u = u_at_t1 + params_.cruise_vel * (t - t1);
  } else {
    // Phase 3: Deceleration
    const double tau = t - t2;
    state.u_ddot = -params_.accel;
    state.u_dot = params_.cruise_vel - params_.accel * tau;
    const double time_remaining = params_.t_decel - tau;
    state.u = 1.0 - 0.5 * params_.accel * time_remaining * time_remaining;
  }

  state.u = std::clamp(state.u, 0.0, 1.0);
  return state;
}

// ===========================================================================
// 5. QUINTIC POLYNOMIAL PROFILER IMPLEMENTATION
// ===========================================================================

double QuinticPolynomialProfiler::computeProfile(
  const IPathInterpolationStrategy & path,
  const KinematicLimits & limits)
{
  const size_t dof = path.getDOF();
  if (!limits.isValid(dof)) {
    throw std::invalid_argument(
      "QuinticPolynomialProfiler::computeProfile: Invalid kinematic limits dimensions or values.");
  }

  constexpr size_t NUM_SAMPLES = 101;
  Eigen::VectorXd peak_dS_du = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd peak_d2S_du2 = Eigen::VectorXd::Zero(dof);

  for (size_t s = 0; s < NUM_SAMPLES; ++s) {
    const double u_sample = static_cast<double>(s) / static_cast<double>(NUM_SAMPLES - 1);
    const PathEvaluation eval = path.evaluate(u_sample);
    for (size_t i = 0; i < dof; ++i) {
      peak_dS_du[i] = std::max(peak_dS_du[i], std::abs(eval.velocity_du[i]));
      peak_d2S_du2[i] = std::max(peak_d2S_du2[i], std::abs(eval.accel_du2[i]));
    }
  }

  double u_dot_limit = 1e6;
  double u_ddot_limit = 1e6;
  for (size_t i = 0; i < dof; ++i) {
    const double ds = std::max(peak_dS_du[i], 1e-4);
    u_dot_limit = std::min(u_dot_limit, limits.max_velocities[i] / ds);
    u_ddot_limit = std::min(u_ddot_limit, (0.60 * limits.max_accelerations[i]) / ds);
    if (peak_d2S_du2[i] > 1e-4) {
      const double v_curv = std::sqrt((0.40 * limits.max_accelerations[i]) / peak_d2S_du2[i]);
      u_dot_limit = std::min(u_dot_limit, v_curv);
    }
  }

  const double T_vel = 1.875 / std::max(u_dot_limit, 1e-3);
  const double T_acc = std::sqrt(5.7735 / std::max(u_ddot_limit, 1e-3));
  total_time_ = std::max(T_vel, T_acc);

  // Closed-loop verification and scaling
  constexpr size_t VERIFY_SAMPLES = 200;
  double max_vel_ratio = 0.0;
  double max_acc_ratio = 0.0;

  for (size_t s = 0; s <= VERIFY_SAMPLES; ++s) {
    const double t_test = total_time_ * (static_cast<double>(s) / static_cast<double>(VERIFY_SAMPLES));
    const ProfileState st = evaluate(t_test);
    const PathEvaluation geom = path.evaluate(st.u);

    for (size_t i = 0; i < dof; ++i) {
      const double v_joint = std::abs(geom.velocity_du[i] * st.u_dot);
      const double a_joint = std::abs(
        geom.accel_du2[i] * (st.u_dot * st.u_dot) +
        geom.velocity_du[i] * st.u_ddot);

      max_vel_ratio = std::max(max_vel_ratio, v_joint / limits.max_velocities[i]);
      max_acc_ratio = std::max(max_acc_ratio, a_joint / limits.max_accelerations[i]);
    }
  }

  const double dilation = std::max({1.0, max_vel_ratio, std::sqrt(max_acc_ratio)}) * 1.005;
  total_time_ *= dilation;

  return total_time_;
}

ProfileState QuinticPolynomialProfiler::evaluate(double t) const
{
  ProfileState state;
  if (t <= 0.0) {
    state.u = 0.0;
    state.u_dot = 0.0;
    state.u_ddot = 0.0;
    return state;
  }
  if (t >= total_time_) {
    state.u = 1.0;
    state.u_dot = 0.0;
    state.u_ddot = 0.0;
    return state;
  }

  const double tau = t / total_time_;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  state.u = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  state.u_dot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / total_time_;
  state.u_ddot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (total_time_ * total_time_);

  state.u = std::clamp(state.u, 0.0, 1.0);
  return state;
}

// ===========================================================================
// 6. FACTORY PATTERN IMPLEMENTATION
// ===========================================================================

std::shared_ptr<IPathInterpolationStrategy> TrajectoryStrategyFactory::createInterpolator(
  InterpolatorType type)
{
  switch (type) {
    case InterpolatorType::CUBIC_BSPLINE:
      return std::make_shared<CubicBSplineInterpolator>();
    case InterpolatorType::LINEAR_SEGMENT:
      return std::make_shared<LinearSegmentInterpolator>();
  }
  return std::make_shared<CubicBSplineInterpolator>();
}

std::shared_ptr<ITimeParameterizationStrategy> TrajectoryStrategyFactory::createProfiler(
  ProfilerType type)
{
  switch (type) {
    case ProfilerType::TRAPEZOIDAL:
      return std::make_shared<TrapezoidalVelocityProfiler>();
    case ProfilerType::QUINTIC_POLYNOMIAL:
      return std::make_shared<QuinticPolynomialProfiler>();
  }
  return std::make_shared<TrapezoidalVelocityProfiler>();
}

// ===========================================================================
// 7. MASTER ENGINE IMPLEMENTATION: TrajectoryPlanningEngine
// ===========================================================================

TrajectoryPlanningEngine::TrajectoryPlanningEngine(
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & max_velocities,
  const Eigen::VectorXd & max_accelerations)
: joint_names_(joint_names),
  limits_{max_velocities, max_accelerations},
  interpolation_strategy_(TrajectoryStrategyFactory::createInterpolator(InterpolatorType::CUBIC_BSPLINE)),
  time_strategy_(TrajectoryStrategyFactory::createProfiler(ProfilerType::TRAPEZOIDAL))
{
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setInterpolator(InterpolatorType type)
{
  return setInterpolationStrategy(TrajectoryStrategyFactory::createInterpolator(type));
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setInterpolationStrategy(
  std::shared_ptr<IPathInterpolationStrategy> strategy) noexcept
{
  if (strategy) {
    interpolation_strategy_ = std::move(strategy);
  }
  return *this;
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setProfiler(ProfilerType type)
{
  return setTimeParameterizationStrategy(TrajectoryStrategyFactory::createProfiler(type));
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setTimeParameterizationStrategy(
  std::shared_ptr<ITimeParameterizationStrategy> strategy) noexcept
{
  if (strategy) {
    time_strategy_ = std::move(strategy);
  }
  return *this;
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setJointLimits(
  const Eigen::VectorXd & max_vel,
  const Eigen::VectorXd & max_acc)
{
  if (static_cast<size_t>(max_vel.size()) != joint_names_.size() ||
      static_cast<size_t>(max_acc.size()) != joint_names_.size())
  {
    throw std::invalid_argument(
      "TrajectoryPlanningEngine::setJointLimits: Dimension mismatch with joint names.");
  }
  limits_.max_velocities = max_vel;
  limits_.max_accelerations = max_acc;
  return *this;
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setKinematicLimits(KinematicLimits limits)
{
  if (!limits.isValid(joint_names_.size())) {
    throw std::invalid_argument(
      "TrajectoryPlanningEngine::setKinematicLimits: Invalid kinematic limits dimension or values.");
  }
  limits_ = std::move(limits);
  return *this;
}

TrajectoryPlanningEngine & TrajectoryPlanningEngine::setJointNames(std::vector<std::string> names)
{
  joint_names_ = std::move(names);
  return *this;
}

double TrajectoryPlanningEngine::computeDuration(
  const std::vector<Eigen::VectorXd> & waypoints) const
{
  interpolation_strategy_->fit(waypoints);
  return time_strategy_->computeProfile(*interpolation_strategy_, limits_);
}

JointTrajectoryState TrajectoryPlanningEngine::sampleAt(double t) const
{
  const ProfileState profile = time_strategy_->evaluate(t);
  const PathEvaluation geom = interpolation_strategy_->evaluate(profile.u);

  JointTrajectoryState state;
  state.position = geom.position;
  state.velocity = geom.velocity_du * profile.u_dot;
  state.acceleration = geom.accel_du2 * (profile.u_dot * profile.u_dot) +
                       geom.velocity_du * profile.u_ddot;
  state.time_from_start = t;
  return state;
}

TrajectoryUtilizationReport TrajectoryPlanningEngine::evaluateUtilization(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  const size_t dof = joint_names_.size();
  TrajectoryUtilizationReport rep;
  rep.peak_velocities = Eigen::VectorXd::Zero(dof);
  rep.peak_accelerations = Eigen::VectorXd::Zero(dof);
  rep.velocity_utilization = Eigen::VectorXd::Zero(dof);
  rep.accel_utilization = Eigen::VectorXd::Zero(dof);
  rep.limits_satisfied = true;

  for (const auto & pt : trajectory.points) {
    for (size_t i = 0; i < dof; ++i) {
      const double v = std::abs(pt.velocities[i]);
      const double a = std::abs(pt.accelerations[i]);
      rep.peak_velocities[i] = std::max(rep.peak_velocities[i], v);
      rep.peak_accelerations[i] = std::max(rep.peak_accelerations[i], a);
    }
  }

  for (size_t i = 0; i < dof; ++i) {
    rep.velocity_utilization[i] = rep.peak_velocities[i] / limits_.max_velocities[i];
    rep.accel_utilization[i] = rep.peak_accelerations[i] / limits_.max_accelerations[i];
    if (rep.velocity_utilization[i] > 1.001 || rep.accel_utilization[i] > 1.001) {
      rep.limits_satisfied = false;
    }
  }

  return rep;
}

TrajectoryResult TrajectoryPlanningEngine::plan(
  const std::vector<Eigen::VectorXd> & waypoints,
  double sample_dt,
  const std::string & frame_id) const
{
  TrajectoryResult result;
  if (waypoints.empty()) {
    result.success = false;
    result.status_message = "Failed: Waypoints sequence is empty.";
    return result;
  }
  if (sample_dt <= 1e-4) {
    result.success = false;
    result.status_message = "Failed: sample_dt must be greater than 0.0001s.";
    return result;
  }

  // 1. Fit geometric curve S(u)
  interpolation_strategy_->fit(waypoints);

  // 2. Compute timing u(t)
  const double total_time = time_strategy_->computeProfile(
    *interpolation_strategy_, limits_);

  // 3. Assemble ROS 2 JointTrajectory message
  result.trajectory.header.frame_id = frame_id;
  result.trajectory.joint_names = joint_names_;

  const size_t dof = joint_names_.size();
  double current_time = 0.0;

  while (current_time <= total_time) {
    const JointTrajectoryState state = sampleAt(current_time);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(state.position.data(), state.position.data() + dof);
    point.velocities.assign(state.velocity.data(), state.velocity.data() + dof);
    point.accelerations.assign(state.acceleration.data(), state.acceleration.data() + dof);

    const int64_t nanoseconds = static_cast<int64_t>(std::round(current_time * 1e9));
    point.time_from_start = rclcpp::Duration::from_nanoseconds(nanoseconds);

    result.trajectory.points.push_back(point);
    current_time += sample_dt;
  }

  // Ensure exact final endpoint at t = total_time is included
  if (result.trajectory.points.empty() || current_time - sample_dt < total_time - 1e-6) {
    const JointTrajectoryState final_state = sampleAt(total_time);

    trajectory_msgs::msg::JointTrajectoryPoint final_pt;
    final_pt.positions.assign(final_state.position.data(), final_state.position.data() + dof);
    final_pt.velocities.assign(final_state.velocity.data(), final_state.velocity.data() + dof);
    final_pt.accelerations.assign(final_state.acceleration.data(), final_state.acceleration.data() + dof);

    const int64_t nanoseconds = static_cast<int64_t>(std::round(total_time * 1e9));
    final_pt.time_from_start = rclcpp::Duration::from_nanoseconds(nanoseconds);

    result.trajectory.points.push_back(final_pt);
  }

  result.duration = total_time;
  result.num_points = result.trajectory.points.size();
  result.report = evaluateUtilization(result.trajectory);
  result.success = result.report.limits_satisfied;
  result.status_message = result.success ?
    "Trajectory planning and time parameterization successful." :
    "Trajectory planned but limits exceeded.";

  return result;
}

trajectory_msgs::msg::JointTrajectory TrajectoryPlanningEngine::parameterize(
  const std::vector<Eigen::VectorXd> & waypoints,
  double sample_dt,
  const std::string & frame_id) const
{
  return plan(waypoints, sample_dt, frame_id).trajectory;
}

}  // namespace dobot_trajectory
