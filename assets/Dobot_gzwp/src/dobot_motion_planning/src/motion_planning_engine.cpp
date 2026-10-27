#include "dobot_motion_planning/motion_planning_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dobot_motion_planning
{

// ===========================================================================
// GEOMETRIC HELPER: POINT-TO-SEGMENT DISTANCE
// ===========================================================================

double MotionPlanningEngine::distancePointToSegment(
  const Eigen::Vector3d & pt,
  const Eigen::Vector3d & seg_a,
  const Eigen::Vector3d & seg_b)
{
  const Eigen::Vector3d v = seg_b - seg_a;
  const double v_sq = v.squaredNorm();
  if (v_sq < 1e-12) {
    return (pt - seg_a).norm();
  }
  const double t = std::clamp((pt - seg_a).dot(v) / v_sq, 0.0, 1.0);
  const Eigen::Vector3d proj = seg_a + t * v;
  return (pt - proj).norm();
}

double MotionPlanningEngine::computePathLength(const std::vector<Eigen::VectorXd> & path) noexcept
{
  if (path.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

// ===========================================================================
// 1. CONCRETE OBSTACLE IMPLEMENTATIONS (OPEN/CLOSED PRINCIPLE)
// ===========================================================================

double SphereObstacle::distanceToSegment(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b) const noexcept
{
  const double dist_to_center = MotionPlanningEngine::distancePointToSegment(center, a, b);
  return dist_to_center - radius;
}

double BoxObstacle::distanceToSegment(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b) const noexcept
{
  const double seg_len = (b - a).norm();
  const size_t num_samples = std::max<size_t>(5, static_cast<size_t>(seg_len / 0.02) + 1);

  double min_dist_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i <= num_samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_samples);
    const Eigen::Vector3d pt = a + t * (b - a);

    const double dx = std::max(0.0, std::max(min_corner.x() - pt.x(), pt.x() - max_corner.x()));
    const double dy = std::max(0.0, std::max(min_corner.y() - pt.y(), pt.y() - max_corner.y()));
    const double dz = std::max(0.0, std::max(min_corner.z() - pt.z(), pt.z() - max_corner.z()));
    const double d_sq = dx * dx + dy * dy + dz * dz;
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
    }
  }
  return std::sqrt(min_dist_sq);
}

double CylinderObstacle::distanceToSegment(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b) const noexcept
{
  const double seg_len = (b - a).norm();
  const size_t num_samples = std::max<size_t>(5, static_cast<size_t>(seg_len / 0.02) + 1);

  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i <= num_samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_samples);
    const Eigen::Vector3d pt = a + t * (b - a);

    const Eigen::Vector3d rel_pt = pt - base_center;
    const double h_proj = rel_pt.dot(axis);
    const double h_clamped = std::clamp(h_proj, 0.0, height);
    const Eigen::Vector3d axis_pt = base_center + h_clamped * axis;
    const double radial_dist = (pt - axis_pt).norm();
    const double dist = radial_dist - radius;

    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return min_dist;
}

// ===========================================================================
// 2. CONCRETE PLANNER STRATEGY 1: RRT
// ===========================================================================

size_t RRTPlannerStrategy::findNearest(
  const std::vector<RRTNode> & tree,
  const Eigen::VectorXd & target) const noexcept
{
  size_t nearest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double d_sq = (tree[i].q - target).squaredNorm();
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

Eigen::VectorXd RRTPlannerStrategy::steer(
  const Eigen::VectorXd & from,
  const Eigen::VectorXd & to,
  double max_step) const noexcept
{
  const Eigen::VectorXd diff = to - from;
  const double dist_sq = diff.squaredNorm();
  if (dist_sq <= max_step * max_step || dist_sq < 1e-16) {
    return to;
  }
  return from + (max_step / std::sqrt(dist_sq)) * diff;
}

std::vector<Eigen::VectorXd> RRTPlannerStrategy::extractPath(
  const std::vector<RRTNode> & tree,
  int goal_idx,
  const Eigen::VectorXd & goal_q) const
{
  std::vector<Eigen::VectorXd> path;
  int curr = goal_idx;
  while (curr >= 0) {
    path.push_back(tree[curr].q);
    curr = tree[curr].parent_idx;
  }
  std::reverse(path.begin(), path.end());
  path.push_back(goal_q);
  return path;
}

PlanningResult RRTPlannerStrategy::plan(
  const MotionPlanningEngine & context,
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q,
  const PlannerConfig & config)
{
  const auto start_time = std::chrono::steady_clock::now();
  PlanningResult result;
  result.planner_name = getName();

  if (context.isSegmentValid(start_q, goal_q, config.step_size)) {
    result.success = true;
    result.path = {start_q, goal_q};
    result.path_length = (goal_q - start_q).norm();
    result.nodes_explored = 2;
    result.status_message = "Direct line-of-sight path connected without collision.";
    const auto end_time = std::chrono::steady_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    return result;
  }

  std::mt19937 rng(config.random_seed);

  std::vector<RRTNode> tree;
  tree.reserve(config.max_iterations);
  tree.push_back({start_q, -1, 0.0});

  int goal_node_idx = -1;

  for (size_t iter = 0; iter < config.max_iterations; ++iter) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    if (elapsed > config.timeout_seconds) {
      break;
    }

    const Eigen::VectorXd q_rand = context.sampleUniform(goal_q, config.goal_bias, rng);
    const size_t nearest_idx = findNearest(tree, q_rand);
    const Eigen::VectorXd & q_near = tree[nearest_idx].q;
    const Eigen::VectorXd q_new = steer(q_near, q_rand, config.step_size);

    if ((q_new - q_near).squaredNorm() < 1e-12) {
      continue;
    }

    if (context.isSegmentValid(q_near, q_new, 0.02)) {
      const double step_cost = (q_new - q_near).norm();
      tree.push_back({q_new, static_cast<int>(nearest_idx), tree[nearest_idx].cost + step_cost});
      const int new_idx = static_cast<int>(tree.size() - 1);

      if ((q_new - goal_q).norm() < config.goal_tolerance ||
          context.isSegmentValid(q_new, goal_q, config.step_size)) {
        goal_node_idx = new_idx;
        break;
      }
    }
  }

  const auto end_time = std::chrono::steady_clock::now();
  result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  result.nodes_explored = tree.size();

  if (goal_node_idx >= 0) {
    result.success = true;
    result.path = extractPath(tree, goal_node_idx, goal_q);
    result.path_length = MotionPlanningEngine::computePathLength(result.path);
    result.status_message = "Path successfully found and verified.";
  } else {
    result.success = false;
    result.status_message = "Planning failed: Timeout or max iterations reached without goal connection.";
  }

  return result;
}

// ===========================================================================
// 3. CONCRETE PLANNER STRATEGY 2: RRT*
// ===========================================================================

size_t RRTStarPlannerStrategy::findNearest(
  const std::vector<RRTNode> & tree,
  const Eigen::VectorXd & target) const noexcept
{
  size_t nearest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double d_sq = (tree[i].q - target).squaredNorm();
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

Eigen::VectorXd RRTStarPlannerStrategy::steer(
  const Eigen::VectorXd & from,
  const Eigen::VectorXd & to,
  double max_step) const noexcept
{
  const Eigen::VectorXd diff = to - from;
  const double dist_sq = diff.squaredNorm();
  if (dist_sq <= max_step * max_step || dist_sq < 1e-16) {
    return to;
  }
  return from + (max_step / std::sqrt(dist_sq)) * diff;
}

std::vector<int> RRTStarPlannerStrategy::findNearNeighbors(
  const std::vector<RRTNode> & tree,
  const Eigen::VectorXd & center,
  double radius) const
{
  const double radius_sq = radius * radius;
  std::vector<int> near_indices;
  near_indices.reserve(32);
  for (size_t i = 0; i < tree.size(); ++i) {
    if ((tree[i].q - center).squaredNorm() <= radius_sq) {
      near_indices.push_back(static_cast<int>(i));
    }
  }
  return near_indices;
}

int RRTStarPlannerStrategy::chooseBestParent(
  const MotionPlanningEngine & ctx,
  const std::vector<RRTNode> & tree,
  const std::vector<int> & near_indices,
  const Eigen::VectorXd & q_new,
  double & best_cost) const
{
  int best_parent = -1;
  for (int idx : near_indices) {
    const double c = tree[idx].cost + (q_new - tree[idx].q).norm();
    if (c < best_cost) {
      if (ctx.isSegmentValid(tree[idx].q, q_new, 0.02)) {
        best_cost = c;
        best_parent = idx;
      }
    }
  }
  return best_parent;
}

void RRTStarPlannerStrategy::rewireNeighbors(
  const MotionPlanningEngine & ctx,
  std::vector<RRTNode> & tree,
  int new_idx,
  const std::vector<int> & near_indices) const
{
  const Eigen::VectorXd & q_new = tree[new_idx].q;
  for (int idx : near_indices) {
    if (idx == tree[new_idx].parent_idx) {
      continue;
    }
    const double candidate_cost = tree[new_idx].cost + (tree[idx].q - q_new).norm();
    if (candidate_cost < tree[idx].cost) {
      if (ctx.isSegmentValid(q_new, tree[idx].q, 0.02)) {
        tree[idx].parent_idx = new_idx;
        tree[idx].cost = candidate_cost;
      }
    }
  }
}

std::vector<Eigen::VectorXd> RRTStarPlannerStrategy::extractPath(
  const std::vector<RRTNode> & tree,
  int goal_idx,
  const Eigen::VectorXd & goal_q) const
{
  std::vector<Eigen::VectorXd> path;
  int curr = goal_idx;
  while (curr >= 0) {
    path.push_back(tree[curr].q);
    curr = tree[curr].parent_idx;
  }
  std::reverse(path.begin(), path.end());
  path.push_back(goal_q);
  return path;
}

PlanningResult RRTStarPlannerStrategy::plan(
  const MotionPlanningEngine & context,
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q,
  const PlannerConfig & config)
{
  const auto start_time = std::chrono::steady_clock::now();
  PlanningResult result;
  result.planner_name = getName();

  if (context.isSegmentValid(start_q, goal_q, config.step_size)) {
    result.success = true;
    result.path = {start_q, goal_q};
    result.path_length = (goal_q - start_q).norm();
    result.nodes_explored = 2;
    result.status_message = "Direct line-of-sight path connected without collision.";
    const auto end_time = std::chrono::steady_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    return result;
  }

  std::mt19937 rng(config.random_seed);

  std::vector<RRTNode> tree;
  tree.reserve(config.max_iterations);
  tree.push_back({start_q, -1, 0.0});

  int best_goal_idx = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();

  for (size_t iter = 0; iter < config.max_iterations; ++iter) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    if (elapsed > config.timeout_seconds) {
      break;
    }

    const Eigen::VectorXd q_rand = context.sampleUniform(goal_q, config.goal_bias, rng);
    const size_t nearest_idx = findNearest(tree, q_rand);
    const Eigen::VectorXd & q_near = tree[nearest_idx].q;
    const Eigen::VectorXd q_new = steer(q_near, q_rand, config.step_size);

    if ((q_new - q_near).squaredNorm() < 1e-12) {
      continue;
    }

    if (!context.isSegmentValid(q_near, q_new, 0.02)) {
      continue;
    }

    const std::vector<int> near_indices = findNearNeighbors(tree, q_new, config.rrt_star_radius);
    double min_cost = tree[nearest_idx].cost + (q_new - q_near).norm();
    int best_parent = static_cast<int>(nearest_idx);

    const int optimal_parent = chooseBestParent(context, tree, near_indices, q_new, min_cost);
    if (optimal_parent >= 0) {
      best_parent = optimal_parent;
    }

    tree.push_back({q_new, best_parent, min_cost});
    const int new_idx = static_cast<int>(tree.size() - 1);

    rewireNeighbors(context, tree, new_idx, near_indices);

    const double dist_to_goal = (q_new - goal_q).norm();
    const double candidate_goal_cost = tree[new_idx].cost + dist_to_goal;
    if (candidate_goal_cost < best_goal_cost) {
      if (dist_to_goal < config.goal_tolerance ||
          context.isSegmentValid(q_new, goal_q, config.step_size)) {
        best_goal_cost = candidate_goal_cost;
        best_goal_idx = new_idx;
      }
    }
  }

  const auto end_time = std::chrono::steady_clock::now();
  result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  result.nodes_explored = tree.size();

  if (best_goal_idx >= 0) {
    result.success = true;
    result.path = extractPath(tree, best_goal_idx, goal_q);
    result.path_length = MotionPlanningEngine::computePathLength(result.path);
    result.status_message = "Optimal path successfully found and rewired.";
  } else {
    result.success = false;
    result.status_message = "RRT* failed to connect within iteration threshold.";
  }

  return result;
}

// ===========================================================================
// 4. CONCRETE PLANNER STRATEGY 3: APF
// ===========================================================================

Eigen::VectorXd APFPlannerStrategy::computeAttractiveForce(
  const Eigen::VectorXd & q_curr,
  const Eigen::VectorXd & q_goal) const noexcept
{
  Eigen::VectorXd f_att = k_att_ * (q_goal - q_curr);
  const double norm = f_att.norm();
  if (norm > 1.0) {
    f_att = f_att.normalized();
  }
  return f_att;
}

Eigen::VectorXd APFPlannerStrategy::computeRepulsiveForce(
  const Eigen::VectorXd & q_curr,
  const Eigen::VectorXd & q_goal,
  double clearance,
  const Eigen::VectorXd & grad_c) const noexcept
{
  if (clearance >= d_influence_ || clearance <= 0.001) {
    return Eigen::VectorXd::Zero(q_curr.size());
  }

  const double grad_norm = grad_c.norm();
  if (grad_norm < 1e-6) {
    return Eigen::VectorXd::Zero(q_curr.size());
  }

  const double dist_to_goal = (q_curr - q_goal).norm();
  const double goal_scaling = std::min(1.0, dist_to_goal / d_influence_);
  double rep_magnitude = k_rep_ * (1.0 / clearance - 1.0 / d_influence_) /
                         (clearance * clearance) * goal_scaling;
  rep_magnitude = std::min(rep_magnitude, 3.0);

  return rep_magnitude * (grad_c / grad_norm);
}

Eigen::VectorXd APFPlannerStrategy::computeTangentialVortex(
  const Eigen::VectorXd & F_att,
  const Eigen::VectorXd & F_rep,
  size_t dof) const noexcept
{
  const double f_att_norm = F_att.norm();
  if (F_rep.norm() <= 0.05 || f_att_norm <= 1e-4) {
    return Eigen::VectorXd::Zero(dof);
  }

  const Eigen::VectorXd u_att = F_att / f_att_norm;
  const double alignment = (F_rep / F_rep.norm()).dot(u_att);
  if (alignment >= -0.2) {
    return Eigen::VectorXd::Zero(dof);
  }

  Eigen::VectorXd e_pitch = Eigen::VectorXd::Zero(dof);
  if (dof > 2) {
    e_pitch[1] = 0.707;
    e_pitch[2] = 0.707;
  } else {
    e_pitch[0] = 1.0;
  }

  Eigen::VectorXd v_ortho = e_pitch - (e_pitch.dot(u_att)) * u_att;
  if (v_ortho.norm() > 1e-3) {
    v_ortho.normalize();
    return 1.2 * F_rep.norm() * v_ortho;
  }

  return Eigen::VectorXd::Zero(dof);
}

Eigen::VectorXd APFPlannerStrategy::computeClearanceGradient(
  const MotionPlanningEngine & ctx,
  const Eigen::VectorXd & q_curr,
  double eps) const
{
  const size_t dof = q_curr.size();
  Eigen::VectorXd grad_c = Eigen::VectorXd::Zero(dof);
  for (size_t d = 0; d < dof; ++d) {
    Eigen::VectorXd q_plus = q_curr;
    Eigen::VectorXd q_minus = q_curr;
    q_plus[d] += eps;
    q_minus[d] -= eps;
    grad_c[d] = (ctx.getClearance(q_plus) - ctx.getClearance(q_minus)) / (2.0 * eps);
  }
  return grad_c;
}

bool APFPlannerStrategy::attemptEscapeStep(
  const MotionPlanningEngine & ctx,
  Eigen::VectorXd & q_curr,
  double step_size,
  std::mt19937 & rng) const
{
  const size_t dof = q_curr.size();
  std::uniform_real_distribution<double> rand_step(-step_size, step_size);

  for (int attempt = 0; attempt < 25; ++attempt) {
    Eigen::VectorXd escape(dof);
    for (size_t d = 0; d < dof; ++d) {
      escape[d] = rand_step(rng);
    }
    const Eigen::VectorXd q_cand = q_curr + escape;
    if (ctx.isStateValid(q_cand) && ctx.isSegmentValid(q_curr, q_cand, 0.02)) {
      q_curr = q_cand;
      return true;
    }
  }
  return false;
}

PlanningResult APFPlannerStrategy::plan(
  const MotionPlanningEngine & context,
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q,
  const PlannerConfig & config)
{
  const auto start_time = std::chrono::steady_clock::now();
  PlanningResult result;
  result.planner_name = getName();

  const size_t dof = start_q.size();
  std::vector<Eigen::VectorXd> path;
  path.push_back(start_q);

  Eigen::VectorXd q_curr = start_q;
  std::mt19937 rng(config.random_seed);
  const double eps = 1e-4;
  size_t local_minima_stagnation = 0;

  for (size_t iter = 0; iter < config.max_iterations; ++iter) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    if (elapsed > config.timeout_seconds) {
      break;
    }

    const double dist_to_goal = (q_curr - goal_q).norm();
    if (dist_to_goal < config.goal_tolerance) {
      path.push_back(goal_q);
      result.success = true;
      break;
    }

    if (dist_to_goal < 0.5 && context.isSegmentValid(q_curr, goal_q, config.step_size)) {
      path.push_back(goal_q);
      result.success = true;
      break;
    }

    const Eigen::VectorXd F_att = computeAttractiveForce(q_curr, goal_q);
    const double clearance = context.getClearance(q_curr);
    const Eigen::VectorXd grad_c = computeClearanceGradient(context, q_curr, eps);
    const Eigen::VectorXd F_rep = computeRepulsiveForce(q_curr, goal_q, clearance, grad_c);
    const Eigen::VectorXd F_vortex = computeTangentialVortex(F_att, F_rep, dof);

    Eigen::VectorXd F_total = F_att + F_rep + F_vortex;

    if (local_minima_stagnation > 2 || (F_total.norm() < 0.05 && dist_to_goal > config.goal_tolerance)) {
      if (attemptEscapeStep(context, q_curr, config.step_size * 2.0, rng)) {
        path.push_back(q_curr);
        local_minima_stagnation = 0;
        continue;
      }
    }

    const double f_norm = F_total.norm();
    if (f_norm < 1e-9) {
      ++local_minima_stagnation;
      continue;
    }

    const double step_len = std::min(config.step_size, f_norm * config.step_size);
    const Eigen::VectorXd q_next = q_curr + (step_len / f_norm) * F_total;

    if (context.isSegmentValid(q_curr, q_next, 0.02)) {
      q_curr = q_next;
      path.push_back(q_curr);
      local_minima_stagnation = 0;
    } else {
      ++local_minima_stagnation;
      if (attemptEscapeStep(context, q_curr, config.step_size, rng)) {
        path.push_back(q_curr);
        local_minima_stagnation = 0;
      }
    }
  }

  const auto end_time = std::chrono::steady_clock::now();
  result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  result.nodes_explored = path.size();

  if (result.success) {
    result.path = std::move(path);
    result.path_length = MotionPlanningEngine::computePathLength(result.path);
    result.status_message = "APF reached goal successfully.";
  } else {
    result.status_message = "APF trapped in local minimum or exceeded maximum steps.";
  }

  return result;
}

// ===========================================================================
// 5. CONCRETE OPTIMIZER STRATEGIES
// ===========================================================================

std::vector<Eigen::VectorXd> ShortcutOptimizerStrategy::optimize(
  const MotionPlanningEngine & context,
  const std::vector<Eigen::VectorXd> & path) const
{
  if (path.size() <= 2) {
    return path;
  }

  std::vector<Eigen::VectorXd> smoothed = path;
  std::mt19937 rng(seed_);

  for (size_t iter = 0; iter < iterations_ && smoothed.size() > 2; ++iter) {
    std::uniform_int_distribution<size_t> dist_idx(0, smoothed.size() - 1);
    size_t i = dist_idx(rng);
    size_t j = dist_idx(rng);

    if (i > j) {
      std::swap(i, j);
    }
    if (j - i <= 1) {
      continue;
    }

    if (context.isSegmentValid(smoothed[i], smoothed[j], resolution_)) {
      smoothed.erase(smoothed.begin() + i + 1, smoothed.begin() + j);
    }
  }

  return smoothed;
}

std::vector<Eigen::VectorXd> SubdivisionSplineOptimizerStrategy::optimize(
  const MotionPlanningEngine & context,
  const std::vector<Eigen::VectorXd> & path) const
{
  if (path.size() <= 2) {
    return path;
  }

  std::vector<Eigen::VectorXd> current = path;

  for (size_t level = 0; level < levels_; ++level) {
    if (current.size() < 2) {
      break;
    }

    std::vector<Eigen::VectorXd> next_curve;
    next_curve.reserve(current.size() * 2);
    next_curve.push_back(current.front());

    for (size_t i = 0; i + 1 < current.size(); ++i) {
      const Eigen::VectorXd q_pt = 0.75 * current[i] + 0.25 * current[i + 1];
      const Eigen::VectorXd r_pt = 0.25 * current[i] + 0.75 * current[i + 1];

      if (context.isStateValid(q_pt) && context.isSegmentValid(next_curve.back(), q_pt, resolution_)) {
        next_curve.push_back(q_pt);
      }
      if (context.isStateValid(r_pt) && context.isSegmentValid(next_curve.back(), r_pt, resolution_)) {
        next_curve.push_back(r_pt);
      }
    }

    if (context.isSegmentValid(next_curve.back(), current.back(), resolution_)) {
      next_curve.push_back(current.back());
    } else {
      break;
    }

    current = std::move(next_curve);
  }

  return current;
}

// ===========================================================================
// 6. MASTER UNIFIED CLASS: MotionPlanningEngine
// ===========================================================================

MotionPlanningEngine::MotionPlanningEngine(
  std::shared_ptr<const dobot_kinematics::GeneralizedKinematicsEngine> kinematics_engine,
  double default_link_radius)
: kinematics_engine_(std::move(kinematics_engine)),
  default_link_radius_(default_link_radius)
{
  if (kinematics_engine_) {
    link_radii_.assign(kinematics_engine_->getDOF(), default_link_radius_);
  }
  planner_strategy_ = std::make_shared<RRTPlannerStrategy>();
  optimizer_strategy_ = std::make_shared<ShortcutOptimizerStrategy>();
}

MotionPlanningEngine & MotionPlanningEngine::addObstacle(std::shared_ptr<IObstacle> obstacle)
{
  if (obstacle) {
    obstacles_.push_back(std::move(obstacle));
  }
  return *this;
}

MotionPlanningEngine & MotionPlanningEngine::addSphere(const SphereObstacle & sphere)
{
  return addObstacle(std::make_shared<SphereObstacle>(sphere));
}

MotionPlanningEngine & MotionPlanningEngine::addBox(const BoxObstacle & box)
{
  return addObstacle(std::make_shared<BoxObstacle>(box));
}

MotionPlanningEngine & MotionPlanningEngine::addCylinder(const CylinderObstacle & cylinder)
{
  return addObstacle(std::make_shared<CylinderObstacle>(cylinder));
}

void MotionPlanningEngine::clearObstacles() noexcept
{
  obstacles_.clear();
}

void MotionPlanningEngine::setLinkRadii(std::vector<double> radii)
{
  link_radii_ = std::move(radii);
}

MotionPlanningEngine & MotionPlanningEngine::setKinematicsEngine(
  std::shared_ptr<const dobot_kinematics::GeneralizedKinematicsEngine> engine)
{
  kinematics_engine_ = std::move(engine);
  if (kinematics_engine_) {
    link_radii_.assign(kinematics_engine_->getDOF(), default_link_radius_);
  }
  return *this;
}

const dobot_kinematics::GeneralizedKinematicsEngine & MotionPlanningEngine::getKinematicsEngine() const
{
  if (!kinematics_engine_) {
    throw std::runtime_error("MotionPlanningEngine: Kinematics engine is uninitialized.");
  }
  return *kinematics_engine_;
}

bool MotionPlanningEngine::isStateValid(const Eigen::VectorXd & q) const
{
  if (!kinematics_engine_) {
    return true;
  }

  if (!kinematics_engine_->isWithinLimits(q)) {
    return false;
  }

  const auto chain = kinematics_engine_->computeChain(q);
  const size_t num_links = chain.size() - 1;

  for (size_t i = 0; i < num_links; ++i) {
    const Eigen::Vector3d p_a = chain[i].block<3, 1>(0, 3);
    const Eigen::Vector3d p_b = chain[i + 1].block<3, 1>(0, 3);
    const double r_link = (i < link_radii_.size()) ? link_radii_[i] : default_link_radius_;

    // Single unified polymorphic loop adhering to Open/Closed Principle
    for (const auto & obs : obstacles_) {
      if (obs->distanceToSegment(p_a, p_b) < r_link) {
        return false;
      }
    }
  }

  // Self-collision checking between non-adjacent segments
  for (size_t i = 0; i + 2 < num_links; ++i) {
    const Eigen::Vector3d a1 = chain[i].block<3, 1>(0, 3);
    const Eigen::Vector3d a2 = chain[i + 1].block<3, 1>(0, 3);
    const double r1 = (i < link_radii_.size()) ? link_radii_[i] : default_link_radius_;

    for (size_t j = i + 2; j < num_links; ++j) {
      const Eigen::Vector3d b1 = chain[j].block<3, 1>(0, 3);
      const Eigen::Vector3d b2 = chain[j + 1].block<3, 1>(0, 3);
      const double r2 = (j < link_radii_.size()) ? link_radii_[j] : default_link_radius_;

      const double d = distancePointToSegment(0.5 * (a1 + a2), b1, b2);
      if (d < (r1 + r2)) {
        return false;
      }
    }
  }

  return true;
}

bool MotionPlanningEngine::isSegmentValid(
  const Eigen::VectorXd & q1,
  const Eigen::VectorXd & q2,
  double step_size) const
{
  const double diff = (q2 - q1).norm();
  if (diff < 1e-9) {
    return isStateValid(q1);
  }

  const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(diff / step_size)));
  for (size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    const Eigen::VectorXd q = (1.0 - t) * q1 + t * q2;
    if (!isStateValid(q)) {
      return false;
    }
  }
  return true;
}

double MotionPlanningEngine::getClearance(const Eigen::VectorXd & q) const
{
  if (!kinematics_engine_) {
    return std::numeric_limits<double>::infinity();
  }

  const auto chain = kinematics_engine_->computeChain(q);
  const size_t num_links = chain.size() - 1;
  double min_clearance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < num_links; ++i) {
    const Eigen::Vector3d p_a = chain[i].block<3, 1>(0, 3);
    const Eigen::Vector3d p_b = chain[i + 1].block<3, 1>(0, 3);
    const double r_link = (i < link_radii_.size()) ? link_radii_[i] : default_link_radius_;

    for (const auto & obs : obstacles_) {
      const double c = obs->distanceToSegment(p_a, p_b) - r_link;
      min_clearance = std::min(min_clearance, c);
    }
  }

  return min_clearance;
}

MotionPlanningEngine & MotionPlanningEngine::setPlanner(PlannerType type)
{
  switch (type) {
    case PlannerType::RRT:
      planner_strategy_ = std::make_shared<RRTPlannerStrategy>();
      break;
    case PlannerType::RRT_STAR:
      planner_strategy_ = std::make_shared<RRTStarPlannerStrategy>();
      break;
    case PlannerType::APF:
      planner_strategy_ = std::make_shared<APFPlannerStrategy>();
      break;
  }
  return *this;
}

MotionPlanningEngine & MotionPlanningEngine::setPlannerStrategy(std::shared_ptr<IPlannerStrategy> strategy) noexcept
{
  planner_strategy_ = std::move(strategy);
  return *this;
}

MotionPlanningEngine & MotionPlanningEngine::setOptimizer(OptimizerType type)
{
  switch (type) {
    case OptimizerType::NONE:
      optimizer_strategy_ = std::make_shared<PassthroughOptimizerStrategy>();
      break;
    case OptimizerType::SHORTCUT:
      optimizer_strategy_ = std::make_shared<ShortcutOptimizerStrategy>();
      break;
    case OptimizerType::SUBDIVISION_SPLINE:
      optimizer_strategy_ = std::make_shared<SubdivisionSplineOptimizerStrategy>();
      break;
  }
  return *this;
}

MotionPlanningEngine & MotionPlanningEngine::setOptimizerStrategy(std::shared_ptr<IOptimizerStrategy> strategy) noexcept
{
  optimizer_strategy_ = std::move(strategy);
  return *this;
}

Eigen::VectorXd MotionPlanningEngine::sampleUniform(
  const Eigen::VectorXd & goal_q,
  double goal_bias,
  std::mt19937 & rng) const
{
  std::uniform_real_distribution<double> dist_01(0.0, 1.0);
  if (dist_01(rng) < goal_bias) {
    return goal_q;
  }

  if (kinematics_engine_) {
    const size_t dof = kinematics_engine_->getDOF();
    const auto & j_min = kinematics_engine_->getJointMin();
    const auto & j_max = kinematics_engine_->getJointMax();
    Eigen::VectorXd q(dof);
    for (size_t d = 0; d < dof; ++d) {
      std::uniform_real_distribution<double> dist_d(j_min[d], j_max[d]);
      q[d] = dist_d(rng);
    }
    return q;
  }

  const size_t dof = goal_q.size();
  std::uniform_real_distribution<double> dist_pi(-M_PI, M_PI);
  Eigen::VectorXd q(dof);
  for (size_t d = 0; d < dof; ++d) {
    q[d] = dist_pi(rng);
  }
  return q;
}

PlanningResult MotionPlanningEngine::plan(
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q,
  const PlannerConfig & config) const
{
  PlanningResult invalid_result;

  if (!planner_strategy_) {
    invalid_result.status_message = "MotionPlanningEngine error: Planner strategy is unconfigured.";
    return invalid_result;
  }
  if (start_q.size() != goal_q.size() || start_q.size() == 0) {
    invalid_result.status_message = "Dimension mismatch: start and goal configurations must match in DOF.";
    return invalid_result;
  }
  if (!isStateValid(start_q)) {
    invalid_result.status_message = "Pre-check failure: Start configuration is in collision or out of limits.";
    return invalid_result;
  }
  if (!isStateValid(goal_q)) {
    invalid_result.status_message = "Pre-check failure: Goal configuration is in collision or out of limits.";
    return invalid_result;
  }

  // Execute active planner strategy
  auto result = planner_strategy_->plan(*this, start_q, goal_q, config);

  // Execute active optimizer strategy
  if (result.success && optimizer_strategy_ && result.path.size() > 2) {
    const auto opt_start = std::chrono::steady_clock::now();
    result.path = optimizer_strategy_->optimize(*this, result.path);
    const auto opt_end = std::chrono::steady_clock::now();
    result.planning_time_ms += std::chrono::duration<double, std::milli>(opt_end - opt_start).count();
    result.path_length = computePathLength(result.path);
    result.optimizer_name = optimizer_strategy_->getName();
  } else {
    result.optimizer_name = optimizer_strategy_ ? optimizer_strategy_->getName() : "None";
  }

  return result;
}

PlanningResult MotionPlanningEngine::plan(
  PlannerType planner_type,
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q,
  const PlannerConfig & config)
{
  setPlanner(planner_type);
  return plan(start_q, goal_q, config);
}

}  // namespace dobot_motion_planning
