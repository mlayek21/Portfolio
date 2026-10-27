#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "dobot_kinematics/kinematics_engine.hpp"
#include "dobot_motion_planning/motion_planning_engine.hpp"

namespace
{

void printVector(const std::string & label, const Eigen::VectorXd & v)
{
  std::cout << label << " [";
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::setw(6) << std::fixed << std::setprecision(3) << v[i]
              << (i + 1 < v.size() ? ", " : "]\n");
  }
}

void printCartesianPos(
  const std::string & label,
  const dobot_kinematics::GeneralizedKinematicsEngine & kinematics,
  const Eigen::VectorXd & q)
{
  auto T = kinematics.computeFK(q);
  Eigen::Vector3d p = T.block<3, 1>(0, 3);
  std::cout << label << " [X: " << std::setw(6) << std::fixed << std::setprecision(4) << p.x()
            << ", Y: " << std::setw(6) << p.y()
            << ", Z: " << std::setw(6) << p.z() << "] m\n";
}

void benchmarkPlanner(
  dobot_motion_planning::MotionPlanningEngine & engine,
  dobot_motion_planning::PlannerType planner_type,
  dobot_motion_planning::OptimizerType optimizer_type,
  const Eigen::VectorXd & start_q,
  const Eigen::VectorXd & goal_q)
{
  engine.setPlanner(planner_type);
  engine.setOptimizer(optimizer_type);

  std::cout << "\n=================================================================\n";
  std::cout << "  BENCHMARK: " << engine.getPlannerStrategy().getName() << "\n";
  std::cout << "  OPTIMIZER: " << engine.getOptimizerStrategy().getName() << "\n";
  std::cout << "=================================================================\n";

  dobot_motion_planning::PlannerConfig config;
  config.step_size = 0.08;
  config.goal_bias = 0.20;
  config.rrt_star_radius = 0.50;
  config.max_iterations = 8000;
  config.timeout_seconds = 5.0;

  auto result = engine.plan(start_q, goal_q, config);

  std::cout << "  - Status: " << (result.success ? "SUCCESS" : "FAILED") << "\n";
  std::cout << "  - Diagnostics: " << result.status_message << "\n";
  std::cout << "  - Planning Time: " << std::fixed << std::setprecision(2)
            << result.planning_time_ms << " ms\n";
  std::cout << "  - Nodes Explored: " << result.nodes_explored << "\n";
  std::cout << "  - Active Optimizer: " << result.optimizer_name << "\n";

  if (result.success) {
    std::cout << "  - Path Length (C-space arc): " << std::fixed << std::setprecision(4)
              << result.path_length << " rad\n";
    std::cout << "  - Path Waypoints: " << result.path.size() << " steps\n";

    // Strict path verification: ensure 100% of trajectory is collision-free
    bool all_valid = true;
    for (size_t i = 1; i < result.path.size(); ++i) {
      if (!engine.isSegmentValid(result.path[i - 1], result.path[i], 0.01)) {
        all_valid = false;
        break;
      }
    }

    std::cout << "  - Continuous Collision Verification: "
              << (all_valid ? "PASS (Zero collisions across path)" : "FAIL (Collision detected)") << "\n";

    printVector("  - Waypoint[0] (Start): ", result.path.front());
    printCartesianPos("    Start Cartesian Pose: ", engine.getKinematicsEngine(), result.path.front());

    if (result.path.size() > 2) {
      size_t mid_idx = result.path.size() / 2;
      printVector("  - Waypoint[mid]:        ", result.path[mid_idx]);
      printCartesianPos("    Midpoint Cartesian Pose: ", engine.getKinematicsEngine(), result.path[mid_idx]);
    }

    printVector("  - Waypoint[end] (Goal):  ", result.path.back());
    printCartesianPos("    Goal Cartesian Pose:  ", engine.getKinematicsEngine(), result.path.back());
  }
}

}  // namespace

int main()
{
  std::cout << "#################################################################\n";
  std::cout << "  DOBOT NOVA 5 UNIFIED MOTION PLANNING ENGINE BENCHMARK          \n";
  std::cout << "  Consolidated Master Class: MotionPlanningEngine                \n";
  std::cout << "  Design Patterns: Strategy Pattern & SRP Class Methods          \n";
  std::cout << "#################################################################\n";

  // 1. Initialize Kinematics Engine for Dobot Nova 5 (6-DOF)
  auto kinematics = std::make_shared<dobot_kinematics::GeneralizedKinematicsEngine>();

  // 2. Initialize Master Unified MotionPlanningEngine (holds all kinematics, collisions, & strategies)
  dobot_motion_planning::MotionPlanningEngine engine(kinematics, 0.05);

  // 3. Add Workcell Obstacle (placed directly at trajectory midpoint)
  dobot_motion_planning::SphereObstacle sphere_obs(
    "midpoint_column",
    Eigen::Vector3d(-0.0869, -0.2161, -0.5856),
    0.05);  // 5 cm radius obstacle placed directly at nominal midpoint
  engine.addSphere(sphere_obs);

  std::cout << "\n[Workcell Environment Configured]:\n";
  std::cout << "  - Midpoint Obstacle: Sphere at [X: -0.0869, Y: -0.2161, Z: -0.5856] m, Radius = 0.050 m\n";

  // 4. Define Start and Goal Configurations (Wide swing motion obstructed by midpoint obstacle)
  Eigen::VectorXd q_start(6);
  q_start << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0;
  Eigen::VectorXd q_goal(6);
  q_goal  <<  0.75, -0.30, 0.50, -0.20, 0.40, 0.0;

  std::cout << "\n[Planning Problem Formulation]:\n";
  printVector("Start Joint Angles (q_start): ", q_start);
  printCartesianPos("Start Cartesian Position:     ", *kinematics, q_start);
  std::cout << "  Start State Clearance: " << std::fixed << std::setprecision(4)
            << engine.getClearance(q_start) << " m\n";

  printVector("Goal Joint Angles  (q_goal):  ", q_goal);
  printCartesianPos("Goal Cartesian Position:      ", *kinematics, q_goal);
  std::cout << "  Goal State Clearance:  " << std::fixed << std::setprecision(4)
            << engine.getClearance(q_goal) << " m\n";
  std::cout << "  Nominal Straight-line Path Status: "
            << (engine.isSegmentValid(q_start, q_goal, 0.01) ? "CLEAR (No detour needed)" : "BLOCKED BY OBSTACLE (Detour required)") << "\n";

  // 5. Benchmark 1: RRT + Shortcut Pruning Strategy
  benchmarkPlanner(
    engine,
    dobot_motion_planning::PlannerType::RRT,
    dobot_motion_planning::OptimizerType::SHORTCUT,
    q_start, q_goal);

  // 6. Benchmark 2: RRT* + Shortcut Pruning Strategy
  benchmarkPlanner(
    engine,
    dobot_motion_planning::PlannerType::RRT_STAR,
    dobot_motion_planning::OptimizerType::SHORTCUT,
    q_start, q_goal);

  // 7. Benchmark 3: APF + Subdivision Spline Smoothing Strategy
  benchmarkPlanner(
    engine,
    dobot_motion_planning::PlannerType::APF,
    dobot_motion_planning::OptimizerType::SUBDIVISION_SPLINE,
    q_start, q_goal);

  std::cout << "\n#################################################################\n";
  std::cout << "  UNIFIED MOTION PLANNING ENGINE VERIFICATION COMPLETED!         \n";
  std::cout << "#################################################################\n";

  return 0;
}
