#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "dobot_executive/executive_engine.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

void printStageHeader(const std::string & title)
{
  std::cout << "\n=================================================================\n";
  std::cout << "  TASK STAGE: " << title << "\n";
  std::cout << "=================================================================\n";
}

void printReport(const dobot_executive::TaskExecutionReport & report)
{
  std::cout << "  - Status:              " << (report.success ? "SUCCESS" : "FAILED") << "\n";
  std::cout << "  - Diagnostics:         " << report.status_message << "\n";
  std::cout << "  - IK Computation:      " << std::fixed << std::setprecision(2) << report.ik_time_ms << " ms\n";
  std::cout << "  - Motion Planning Time:" << std::fixed << std::setprecision(2) << report.planning_time_ms << " ms\n";
  std::cout << "  - Waypoints (Path):    " << report.path_waypoints_count << " steps\n";
  std::cout << "  - Trajectory Points:   " << report.trajectory_points_count << " samples (50 Hz)\n";
  std::cout << "  - Trajectory Duration: " << std::fixed << std::setprecision(3) << report.trajectory_duration_s << " s\n";

  const auto & rep = report.utilization;
  if (report.success && rep.velocity_utilization.size() > 0 && rep.accel_utilization.size() > 0) {
    double max_vel_pct = rep.velocity_utilization.maxCoeff() * 100.0;
    double max_acc_pct = rep.accel_utilization.maxCoeff() * 100.0;
    std::cout << "  - Peak Joint Vel Util: " << std::fixed << std::setprecision(1) << max_vel_pct << "%\n";
    std::cout << "  - Peak Joint Acc Util: " << std::fixed << std::setprecision(1) << max_acc_pct << "%\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dobot_executive_node");

  std::cout << "\n#################################################################\n";
  std::cout << "  DOBOT NOVA 5 PHASE 5: END-TO-END MASTER EXECUTIVE ORCHESTRATOR\n";
  std::cout << "  Closed-Loop: IK -> RRT* Detour -> B-Spline -> Robot Controller\n";
  std::cout << "#################################################################\n";

  dobot_executive::ExecutiveConfig config;
  config.auto_execute = true;
  config.sample_dt = 0.02;  // 50 Hz

  dobot_executive::ExecutiveEngine executive(node, config);

  // Configure workcell environment with physical obstacle
  auto obstacle = std::make_shared<dobot_motion_planning::SphereObstacle>(
    "workcell_fixture_obstacle",
    Eigen::Vector3d(0.15, 0.25, 0.85),
    0.050);  // 50mm radius obstacle

  executive.addObstacle(obstacle);
  std::cout << "\n[Workcell Environment]: Sphere obstacle registered at [-0.087, -0.216, -0.586] m, r = 50mm\n";

  // Spin in background thread so ROS subscribers/publishers process events
  std::thread spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Give node 500ms to discover ROS topics
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // =========================================================================
  // TASK STAGE 1: Home to Inspection Pose (Joint Space Planning)
  // =========================================================================
  printStageHeader("1. Articulate Arm to Inspection Pose (Joint Motion)");
  Eigen::VectorXd inspect_joints(6);
  inspect_joints << -0.50, -0.25, 0.40, -0.15, 0.30, 0.00;

  auto report1 = executive.moveJoint(inspect_joints);
  printReport(report1);

  if (report1.success) {
    // Wait for trajectory execution plus buffer
    std::cout << "  Executing trajectory... (Waiting "
              << std::fixed << std::setprecision(2) << report1.trajectory_duration_s << " s)\n";
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((report1.trajectory_duration_s + 0.3) * 1000)));
  }

  // =========================================================================
  // TASK STAGE 2: Obstacle Detour to Goal Pose (RRT* + B-Spline)
  // =========================================================================
  printStageHeader("2. Navigate Detour Across Obstacle to Goal Pose");
  Eigen::VectorXd goal_joints(6);
  goal_joints << 0.75, -0.30, 0.50, -0.20, 0.40, 0.00;

  auto report2 = executive.moveJoint(goal_joints);
  printReport(report2);

  if (report2.success) {
    std::cout << "  Executing obstacle detour trajectory... (Waiting "
              << std::fixed << std::setprecision(2) << report2.trajectory_duration_s << " s)\n";
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((report2.trajectory_duration_s + 0.3) * 1000)));
  }

  // =========================================================================
  // TASK STAGE 3: Cartesian Move with Inverse Kinematics
  // =========================================================================
  printStageHeader("3. Cartesian Motion via Inverse Kinematics (Pick Pose)");
  // Compute reachable Cartesian target pose (Collision-free Pick Location on table)
  Eigen::VectorXd pick_joints(6);
  pick_joints << 1.00, -0.20, 0.40, -0.10, 0.50, 0.0;
  Eigen::Matrix4d T_pick = executive.getKinematics().computeFK(pick_joints);
  Eigen::Vector3d cartesian_goal = T_pick.block<3, 1>(0, 3);
  Eigen::Matrix3d cartesian_rot  = T_pick.block<3, 3>(0, 0);

  auto report3 = executive.moveCartesian(cartesian_goal, cartesian_rot);
  printReport(report3);

  if (report3.success) {
    std::cout << "  Executing Cartesian move... (Waiting "
              << std::fixed << std::setprecision(2) << report3.trajectory_duration_s << " s)\n";
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((report3.trajectory_duration_s + 0.3) * 1000)));
  }

  // =========================================================================
  // TASK STAGE 4: Return to Home
  // =========================================================================
  printStageHeader("4. Return Home Safely");
  Eigen::VectorXd home_joints(6);
  home_joints << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0;

  auto report4 = executive.moveJoint(home_joints);
  printReport(report4);

  if (report4.success) {
    std::cout << "  Returning to home... (Waiting "
              << std::fixed << std::setprecision(2) << report4.trajectory_duration_s << " s)\n";
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((report4.trajectory_duration_s + 0.3) * 1000)));
  }

  std::cout << "\n#################################################################\n";
  std::cout << "  ALL PHASE 5 END-TO-END ORCHESTRATION STAGES COMPLETED!\n";
  std::cout << "#################################################################\n\n";

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
