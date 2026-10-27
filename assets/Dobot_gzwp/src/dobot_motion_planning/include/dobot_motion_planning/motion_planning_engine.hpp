#ifndef DOBOT_MOTION_PLANNING__MOTION_PLANNING_ENGINE_HPP_
#define DOBOT_MOTION_PLANNING__MOTION_PLANNING_ENGINE_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "dobot_kinematics/kinematics_engine.hpp"

namespace dobot_motion_planning
{

// Forward declarations
class MotionPlanningEngine;

// ===========================================================================
// 1. STRATEGY PATTERN: OBSTACLE ABSTRACTION (OPEN/CLOSED PRINCIPLE)
// ===========================================================================

/**
 * @brief Abstract geometric collision obstacle interface.
 * Satisfies Open/Closed Principle: any custom obstacle geometry (Sphere, Box,
 * Cylinder, Mesh, Capsule) can be integrated without modifying the engine.
 */
class IObstacle
{
public:
  virtual ~IObstacle() = default;

  /// Human-readable identifier for telemetry
  virtual std::string getName() const noexcept = 0;

  /**
   * @brief Compute Euclidean distance from line segment [a, b] to obstacle surface.
   * Negative values denote penetration / collision.
   */
  virtual double distanceToSegment(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b) const noexcept = 0;
};

// ===========================================================================
// CONCRETE OBSTACLES
// ===========================================================================

class SphereObstacle : public IObstacle
{
public:
  std::string name;
  Eigen::Vector3d center;
  double radius;

  SphereObstacle(std::string id, const Eigen::Vector3d & c, double r)
  : name(std::move(id)), center(c), radius(r) {}

  std::string getName() const noexcept override { return name; }

  double distanceToSegment(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b) const noexcept override;
};

class BoxObstacle : public IObstacle
{
public:
  std::string name;
  Eigen::Vector3d min_corner;
  Eigen::Vector3d max_corner;

  BoxObstacle(std::string id, const Eigen::Vector3d & min_pt, const Eigen::Vector3d & max_pt)
  : name(std::move(id)), min_corner(min_pt), max_corner(max_pt) {}

  BoxObstacle(std::string id, const Eigen::Vector3d & center, double dx, double dy, double dz)
  : name(std::move(id)),
    min_corner(center - Eigen::Vector3d(dx / 2.0, dy / 2.0, dz / 2.0)),
    max_corner(center + Eigen::Vector3d(dx / 2.0, dy / 2.0, dz / 2.0)) {}

  std::string getName() const noexcept override { return name; }

  double distanceToSegment(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b) const noexcept override;
};

class CylinderObstacle : public IObstacle
{
public:
  std::string name;
  Eigen::Vector3d base_center;
  Eigen::Vector3d axis;
  double radius;
  double height;

  CylinderObstacle(
    std::string id,
    const Eigen::Vector3d & base,
    const Eigen::Vector3d & axis_dir,
    double r,
    double h)
  : name(std::move(id)),
    base_center(base),
    axis(axis_dir.normalized()),
    radius(r),
    height(h) {}

  std::string getName() const noexcept override { return name; }

  double distanceToSegment(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b) const noexcept override;
};

// ===========================================================================
// 2. CONFIGURATION & TELEMETRY
// ===========================================================================

enum class PlannerType
{
  RRT,
  RRT_STAR,
  APF
};

enum class OptimizerType
{
  NONE,
  SHORTCUT,
  SUBDIVISION_SPLINE
};

struct PlannerConfig
{
  double step_size{0.08};             ///< Incremental steering step (radians)
  double goal_bias{0.15};             ///< Probability of sampling q_goal [0.0, 1.0]
  double rrt_star_radius{0.60};       ///< Rewiring radius for RRT* (radians)
  size_t max_iterations{6000};        ///< Max tree expansions / steps
  double timeout_seconds{5.0};        ///< Wall-clock timeout threshold
  double goal_tolerance{0.03};        ///< Distance threshold to goal (radians)
  uint32_t random_seed{1337};         ///< Deterministic seed (thread-safe)
};

struct PlanningResult
{
  bool success{false};
  std::vector<Eigen::VectorXd> path;  ///< Output trajectory waypoints
  double planning_time_ms{0.0};       ///< Execution duration in milliseconds
  double path_length{0.0};            ///< Total C-space path length (radians)
  size_t nodes_explored{0};           ///< Tree nodes instantiated or steps taken
  std::string planner_name;           ///< Active planner strategy
  std::string optimizer_name;         ///< Active optimizer strategy
  std::string status_message;         ///< Diagnostic message
};

struct RRTNode
{
  Eigen::VectorXd q;
  int parent_idx{-1};
  double cost{0.0};
};

// ===========================================================================
// 3. STRATEGY PATTERN: PLANNER STRATEGY INTERFACE
// ===========================================================================

class IPlannerStrategy
{
public:
  virtual ~IPlannerStrategy() = default;
  virtual std::string getName() const noexcept = 0;
  virtual PlanningResult plan(
    const MotionPlanningEngine & context,
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config) = 0;
};

// ===========================================================================
// 4. STRATEGY PATTERN: OPTIMIZER STRATEGY INTERFACE
// ===========================================================================

class IOptimizerStrategy
{
public:
  virtual ~IOptimizerStrategy() = default;
  virtual std::string getName() const noexcept = 0;
  virtual std::vector<Eigen::VectorXd> optimize(
    const MotionPlanningEngine & context,
    const std::vector<Eigen::VectorXd> & path) const = 0;
};

// ===========================================================================
// 5. CONCRETE PLANNER STRATEGIES
// ===========================================================================

class RRTPlannerStrategy : public IPlannerStrategy
{
public:
  std::string getName() const noexcept override { return "Rapidly-exploring Random Tree (RRT)"; }
  PlanningResult plan(
    const MotionPlanningEngine & context,
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config) override;

private:
  size_t findNearest(const std::vector<RRTNode> & tree, const Eigen::VectorXd & target) const noexcept;
  Eigen::VectorXd steer(const Eigen::VectorXd & from, const Eigen::VectorXd & to, double max_step) const noexcept;
  std::vector<Eigen::VectorXd> extractPath(const std::vector<RRTNode> & tree, int goal_idx, const Eigen::VectorXd & goal_q) const;
};

class RRTStarPlannerStrategy : public IPlannerStrategy
{
public:
  std::string getName() const noexcept override { return "Asymptotically Optimal RRT* (Rewiring)"; }
  PlanningResult plan(
    const MotionPlanningEngine & context,
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config) override;

private:
  size_t findNearest(const std::vector<RRTNode> & tree, const Eigen::VectorXd & target) const noexcept;
  Eigen::VectorXd steer(const Eigen::VectorXd & from, const Eigen::VectorXd & to, double max_step) const noexcept;
  std::vector<int> findNearNeighbors(const std::vector<RRTNode> & tree, const Eigen::VectorXd & center, double radius) const;
  int chooseBestParent(const MotionPlanningEngine & ctx, const std::vector<RRTNode> & tree, const std::vector<int> & near_indices, const Eigen::VectorXd & q_new, double & best_cost) const;
  void rewireNeighbors(const MotionPlanningEngine & ctx, std::vector<RRTNode> & tree, int new_idx, const std::vector<int> & near_indices) const;
  std::vector<Eigen::VectorXd> extractPath(const std::vector<RRTNode> & tree, int goal_idx, const Eigen::VectorXd & goal_q) const;
};

class APFPlannerStrategy : public IPlannerStrategy
{
public:
  explicit APFPlannerStrategy(double k_att = 1.0, double k_rep = 0.5, double d_influence = 0.25)
  : k_att_(k_att), k_rep_(k_rep), d_influence_(d_influence) {}

  std::string getName() const noexcept override { return "Artificial Potential Fields (APF)"; }
  PlanningResult plan(
    const MotionPlanningEngine & context,
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config) override;

private:
  double k_att_;
  double k_rep_;
  double d_influence_;

  Eigen::VectorXd computeAttractiveForce(const Eigen::VectorXd & q_curr, const Eigen::VectorXd & q_goal) const noexcept;
  Eigen::VectorXd computeRepulsiveForce(const Eigen::VectorXd & q_curr, const Eigen::VectorXd & q_goal, double clearance, const Eigen::VectorXd & grad_c) const noexcept;
  Eigen::VectorXd computeTangentialVortex(const Eigen::VectorXd & F_att, const Eigen::VectorXd & F_rep, size_t dof) const noexcept;
  Eigen::VectorXd computeClearanceGradient(const MotionPlanningEngine & ctx, const Eigen::VectorXd & q_curr, double eps) const;
  bool attemptEscapeStep(const MotionPlanningEngine & ctx, Eigen::VectorXd & q_curr, double step_size, std::mt19937 & rng) const;
};

// ===========================================================================
// 6. CONCRETE OPTIMIZER STRATEGIES
// ===========================================================================

class ShortcutOptimizerStrategy : public IOptimizerStrategy
{
public:
  explicit ShortcutOptimizerStrategy(size_t iterations = 150, double resolution = 0.012, uint32_t seed = 42)
  : iterations_(iterations), resolution_(resolution), seed_(seed) {}

  std::string getName() const noexcept override { return "Shortcut Pruning Optimizer"; }
  std::vector<Eigen::VectorXd> optimize(
    const MotionPlanningEngine & context,
    const std::vector<Eigen::VectorXd> & path) const override;

private:
  size_t iterations_;
  double resolution_;
  uint32_t seed_;
};

class SubdivisionSplineOptimizerStrategy : public IOptimizerStrategy
{
public:
  explicit SubdivisionSplineOptimizerStrategy(size_t levels = 2, double resolution = 0.012)
  : levels_(levels), resolution_(resolution) {}

  std::string getName() const noexcept override { return "Subdivision Spline Optimizer"; }
  std::vector<Eigen::VectorXd> optimize(
    const MotionPlanningEngine & context,
    const std::vector<Eigen::VectorXd> & path) const override;

private:
  size_t levels_;
  double resolution_;
};

class PassthroughOptimizerStrategy : public IOptimizerStrategy
{
public:
  std::string getName() const noexcept override { return "Passthrough (None)"; }
  std::vector<Eigen::VectorXd> optimize(
    const MotionPlanningEngine & /* context */,
    const std::vector<Eigen::VectorXd> & path) const override
  {
    return path;
  }
};

// ===========================================================================
// 7. UNIFIED MASTER ENGINE CLASS: MotionPlanningEngine
// ===========================================================================

class MotionPlanningEngine
{
public:
  /// Construct with kinematics solver and default link capsule radius
  explicit MotionPlanningEngine(
    std::shared_ptr<const dobot_kinematics::GeneralizedKinematicsEngine> kinematics_engine = nullptr,
    double default_link_radius = 0.05);

  // -------------------------------------------------------------------------
  // Obstacle & Workspace Geometry Management (Polymorphic Extensibility)
  // -------------------------------------------------------------------------
  MotionPlanningEngine & addObstacle(std::shared_ptr<IObstacle> obstacle);
  MotionPlanningEngine & addSphere(const SphereObstacle & sphere);
  MotionPlanningEngine & addBox(const BoxObstacle & box);
  MotionPlanningEngine & addCylinder(const CylinderObstacle & cylinder);
  void clearObstacles() noexcept;
  void setLinkRadii(std::vector<double> radii);

  const std::vector<std::shared_ptr<IObstacle>> & getObstacles() const noexcept { return obstacles_; }

  // -------------------------------------------------------------------------
  // Collision & Kinematic Validation Methods
  // -------------------------------------------------------------------------
  bool isStateValid(const Eigen::VectorXd & q) const;
  bool isSegmentValid(const Eigen::VectorXd & q1, const Eigen::VectorXd & q2, double step_size = 0.02) const;
  double getClearance(const Eigen::VectorXd & q) const;

  // -------------------------------------------------------------------------
  // Strategy Configuration (GoF Strategy & Fluent Method Chaining)
  // -------------------------------------------------------------------------
  MotionPlanningEngine & setPlanner(PlannerType type);
  MotionPlanningEngine & setPlannerStrategy(std::shared_ptr<IPlannerStrategy> strategy) noexcept;
  const IPlannerStrategy & getPlannerStrategy() const noexcept { return *planner_strategy_; }

  MotionPlanningEngine & setOptimizer(OptimizerType type);
  MotionPlanningEngine & setOptimizerStrategy(std::shared_ptr<IOptimizerStrategy> strategy) noexcept;
  const IOptimizerStrategy & getOptimizerStrategy() const noexcept { return *optimizer_strategy_; }

  MotionPlanningEngine & setKinematicsEngine(std::shared_ptr<const dobot_kinematics::GeneralizedKinematicsEngine> engine);
  const dobot_kinematics::GeneralizedKinematicsEngine & getKinematicsEngine() const;

  // -------------------------------------------------------------------------
  // Execution & Planning API (Const-Correct & Thread-Safe)
  // -------------------------------------------------------------------------
  PlanningResult plan(
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config = PlannerConfig{}) const;

  PlanningResult plan(
    PlannerType planner_type,
    const Eigen::VectorXd & start_q,
    const Eigen::VectorXd & goal_q,
    const PlannerConfig & config = PlannerConfig{});

  // -------------------------------------------------------------------------
  // Sampling & Metric Utilities
  // -------------------------------------------------------------------------
  Eigen::VectorXd sampleUniform(
    const Eigen::VectorXd & goal_q,
    double goal_bias,
    std::mt19937 & rng) const;

  static double computePathLength(const std::vector<Eigen::VectorXd> & path) noexcept;
  static double distancePointToSegment(const Eigen::Vector3d & pt, const Eigen::Vector3d & a, const Eigen::Vector3d & b);

private:
  std::shared_ptr<const dobot_kinematics::GeneralizedKinematicsEngine> kinematics_engine_;
  double default_link_radius_{0.05};
  std::vector<double> link_radii_;

  // Polymorphic obstacle collection adhering to Open/Closed Principle
  std::vector<std::shared_ptr<IObstacle>> obstacles_;

  std::shared_ptr<IPlannerStrategy> planner_strategy_;
  std::shared_ptr<IOptimizerStrategy> optimizer_strategy_;
};

}  // namespace dobot_motion_planning

#endif  // DOBOT_MOTION_PLANNING__MOTION_PLANNING_ENGINE_HPP_
