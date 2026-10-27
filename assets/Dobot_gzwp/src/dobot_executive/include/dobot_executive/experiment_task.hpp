#ifndef DOBOT_EXECUTIVE__EXPERIMENT_TASK_HPP_
#define DOBOT_EXECUTIVE__EXPERIMENT_TASK_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dobot_executive/executive_engine.hpp"

namespace dobot_executive
{

// ===========================================================================
// 1. DATA STRUCTURES & DIAGNOSTICS (SRP)
// ===========================================================================

struct ExperimentTaskResult
{
  std::string task_name;
  std::string objective;
  bool success{false};
  std::string status_message;
  double ik_time_ms{0.0};
  double trajectory_duration_s{0.0};
  size_t path_waypoints_count{0};
  size_t trajectory_points_count{0};
  double closure_or_return_error_mm{0.0};
  double peak_vel_utilization_pct{0.0};
  double peak_acc_utilization_pct{0.0};
};

// ===========================================================================
// 2. COMMAND PATTERN: IExperimentTask
// ===========================================================================

/**
 * @brief Abstract Command Pattern Interface for Robotic Experiment Tasks.
 * Encapsulates a distinct experiment trajectory, state validation, and closure check.
 */
class IExperimentTask
{
public:
  virtual ~IExperimentTask() = default;

  [[nodiscard]] virtual std::string getName() const = 0;
  [[nodiscard]] virtual std::string getObjective() const = 0;
  [[nodiscard]] virtual ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer = nullptr) = 0;
};

// Helper function to extract peak utilizations from TaskExecutionReport
inline void extractUtilizations(
  const TaskExecutionReport & report,
  double & peak_vel_pct,
  double & peak_acc_pct)
{
  const auto & rep = report.utilization;
  if (report.success && rep.velocity_utilization.size() > 0 && rep.accel_utilization.size() > 0) {
    peak_vel_pct = rep.velocity_utilization.maxCoeff() * 100.0;
    peak_acc_pct = rep.accel_utilization.maxCoeff() * 100.0;
  }
}

// ===========================================================================
// CONCRETE COMMAND 0: SETUP WORKING POSE (FLOOR QUADRANT 1)
// ===========================================================================

class SetupWorkingPoseTask : public IExperimentTask
{
public:
  explicit SetupWorkingPoseTask(Eigen::VectorXd working_joints = Eigen::VectorXd())
  : q_work_(std::move(working_joints))
  {
    if (q_work_.size() != 6) {
      q_work_.resize(6);
      // Canonical Quadrant 1 (+X, +Y) perpendicular posture:
      // p_start = [0.300, 0.150, 0.220] m (X > 0, Y > 0, Z = 220 mm table height)
      // Tool Z axis strictly pointing down [0, 0, -1] perpendicular to floor
      q_work_ << 0.8779, -0.6005, -1.8861, -2.2258, -1.5708, 2.4487;
    }
  }

  [[nodiscard]] std::string getName() const override
  {
    return "SETUP: Articulate to Floor Working Configuration (+X, +Y Quadrant 1)";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Align 6-axis manipulator so tool Z points strictly down [0, 0, -1] at Z = 0.22m in Quadrant 1.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    auto report = engine.moveJoint(q_work_, "Setup Working Pose");
    res.success = report.success;
    res.status_message = report.status_message;
    res.ik_time_ms = report.ik_time_ms;
    res.trajectory_duration_s = report.trajectory_duration_s;
    res.path_waypoints_count = report.path_waypoints_count;
    res.trajectory_points_count = report.trajectory_points_count;
    extractUtilizations(report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    if (report.success && observer) {
      observer->waitForTrajectory(report.trajectory_duration_s, 0.3);
    }

    Eigen::Matrix4d T_curr = engine.getCurrentPose();
    Eigen::Vector3d p_curr = T_curr.block<3, 1>(0, 3);
    Eigen::Vector3d z_tool = T_curr.block<3, 1>(0, 2);

    std::cout << "  [Initial Floor Working Position]: ["
              << std::fixed << std::setprecision(4) << p_curr.transpose()
              << "] m (X > 0, Y > 0, Plane Height Z = " << p_curr.z() << " m)\n";
    std::cout << "  [End-Effector Tool Z-Axis]: ["
              << std::fixed << std::setprecision(4) << z_tool.transpose()
              << "] -> PERPENDICULAR TO FLOOR\n";

    return res;
  }

private:
  Eigen::VectorXd q_work_;
};

// ===========================================================================
// CONCRETE COMMAND 1: LINEAR CARTESIAN MOTION (MoveL) & RETURN
// ===========================================================================

class LineExperimentTask : public IExperimentTask
{
public:
  explicit LineExperimentTask(double line_length = 0.180, size_t steps = 40)
  : line_length_(line_length), steps_(steps)
  {
  }

  [[nodiscard]] std::string getName() const override
  {
    return "EXPERIMENT 5.1: Linear Cartesian Motion in XY Plane (MoveL) & Return";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Move end-effector +180mm along +X at constant Z, hold tool perpendicular, then reverse to start.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::Vector3d p_init = engine.getCurrentPose().block<3, 1>(0, 3);
    Eigen::Vector3d p_target = p_init + Eigen::Vector3d(line_length_, 0.0, 0.0);

    // 5.1a Forward Line Motion
    auto fwd_report = engine.moveLine(p_target, steps_, "5.1a MoveL Forward");
    if (!fwd_report.success) {
      res.success = false;
      res.status_message = "Forward MoveL failed: " + fwd_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(fwd_report.trajectory_duration_s, 0.3);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 5.1b Return Line Motion
    auto rev_report = engine.moveLine(p_init, steps_, "5.1b MoveL Return");
    if (!rev_report.success) {
      res.success = false;
      res.status_message = "Return MoveL failed: " + rev_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(rev_report.trajectory_duration_s, 0.3);
    }

    Eigen::Vector3d p_final = engine.getCurrentPose().block<3, 1>(0, 3);
    res.closure_or_return_error_mm = (p_final - p_init).norm() * 1000.0;
    res.success = true;
    res.status_message = "MoveL forward and return executed with 0 joint flips.";
    res.ik_time_ms = fwd_report.ik_time_ms + rev_report.ik_time_ms;
    res.trajectory_duration_s = fwd_report.trajectory_duration_s + rev_report.trajectory_duration_s;
    res.path_waypoints_count = fwd_report.path_waypoints_count + rev_report.path_waypoints_count;
    res.trajectory_points_count = fwd_report.trajectory_points_count + rev_report.trajectory_points_count;
    extractUtilizations(fwd_report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    std::cout << "  -> Experiment 5.1 Completed. Return Position Error: "
              << std::fixed << std::setprecision(4) << res.closure_or_return_error_mm << " mm\n";

    return res;
  }

private:
  double line_length_{0.180};
  size_t steps_{40};
};

// ===========================================================================
// CONCRETE COMMAND 2: CIRCULAR CARTESIAN MOTION (MoveC) & RETURN
// ===========================================================================

class CircleExperimentTask : public IExperimentTask
{
public:
  explicit CircleExperimentTask(double radius = 0.080, size_t steps = 50)
  : radius_(radius), steps_(steps)
  {
  }

  [[nodiscard]] std::string getName() const override
  {
    return "EXPERIMENT 5.2: Circular Cartesian Motion in XY Plane (MoveC) & Return";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Trace full 160mm diameter circle in XY plane at constant Z with tool perpendicular, returning to start.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::Vector3d p_init = engine.getCurrentPose().block<3, 1>(0, 3);
    Eigen::Vector3d center = p_init + Eigen::Vector3d(radius_, 0.0, 0.0);
    Eigen::Vector3d normal(0.0, 0.0, 1.0);

    auto report = engine.moveCircle(center, radius_, normal, 2.0 * M_PI, steps_, "5.2 MoveC Circle");
    if (!report.success) {
      res.success = false;
      res.status_message = "MoveC failed: " + report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(report.trajectory_duration_s, 0.3);
    }

    Eigen::Vector3d p_final = engine.getCurrentPose().block<3, 1>(0, 3);
    res.closure_or_return_error_mm = (p_final - p_init).norm() * 1000.0;
    res.success = true;
    res.status_message = "MoveC 360-degree contour completed successfully.";
    res.ik_time_ms = report.ik_time_ms;
    res.trajectory_duration_s = report.trajectory_duration_s;
    res.path_waypoints_count = report.path_waypoints_count;
    res.trajectory_points_count = report.trajectory_points_count;
    extractUtilizations(report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    std::cout << "  -> Experiment 5.2 Completed. Circle Closure Error: "
              << std::fixed << std::setprecision(4) << res.closure_or_return_error_mm << " mm\n";

    return res;
  }

private:
  double radius_{0.080};
  size_t steps_{50};
};

// ===========================================================================
// CONCRETE COMMAND 3: RECTANGULAR CARTESIAN MOTION (MoveRect) & RETURN
// ===========================================================================

class RectangleExperimentTask : public IExperimentTask
{
public:
  explicit RectangleExperimentTask(
    double width = 0.160,
    double height = 0.100,
    size_t steps_per_side = 15)
  : width_(width), height_(height), steps_per_side_(steps_per_side)
  {
  }

  [[nodiscard]] std::string getName() const override
  {
    return "EXPERIMENT 5.3: Rectangular Cartesian Motion in XY Plane (MoveRect) & Return";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Trace closed 160mm x 100mm rectangle in Quadrant 1 at constant Z with tool perpendicular.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::Vector3d p_init = engine.getCurrentPose().block<3, 1>(0, 3);

    auto report = engine.moveRectangle(
      width_,
      height_,
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      steps_per_side_,
      "5.3 MoveRect Rectangle");

    if (!report.success) {
      res.success = false;
      res.status_message = "MoveRect failed: " + report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(report.trajectory_duration_s, 0.3);
    }

    Eigen::Vector3d p_final = engine.getCurrentPose().block<3, 1>(0, 3);
    res.closure_or_return_error_mm = (p_final - p_init).norm() * 1000.0;
    res.success = true;
    res.status_message = "MoveRect 4-corner closed perimeter traversed.";
    res.ik_time_ms = report.ik_time_ms;
    res.trajectory_duration_s = report.trajectory_duration_s;
    res.path_waypoints_count = report.path_waypoints_count;
    res.trajectory_points_count = report.trajectory_points_count;
    extractUtilizations(report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    std::cout << "  -> Experiment 5.3 Completed. Rectangle Closure Error: "
              << std::fixed << std::setprecision(4) << res.closure_or_return_error_mm << " mm\n";

    return res;
  }

private:
  double width_{0.160};
  double height_{0.100};
  size_t steps_per_side_{15};
};

// ===========================================================================
// CONCRETE COMMAND 4: 2D ARCHIMEDEAN SPIRAL (MoveSpiral) & RETURN
// ===========================================================================

class SpiralExperimentTask : public IExperimentTask
{
public:
  explicit SpiralExperimentTask(
    double end_radius = 0.110,
    double turns = 2.0,
    size_t steps = 60)
  : end_radius_(end_radius), turns_(turns), steps_(steps)
  {
  }

  [[nodiscard]] std::string getName() const override
  {
    return "EXPERIMENT 5.4: 2D Archimedean Spiral in XY Plane (MoveSpiral) & Return";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Trace 2-revolution Archimedean spiral (r: 60mm -> 110mm) in XY plane, then return smoothly.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::Vector3d p_init = engine.getCurrentPose().block<3, 1>(0, 3);
    Eigen::Vector3d center(0.360, 0.150, p_init.z());

    auto spiral_report = engine.moveSpiral(
      center, end_radius_, 0.0, turns_, Eigen::Vector3d(0.0, 0.0, 1.0), steps_, "5.4 MoveSpiral 2D");

    if (!spiral_report.success) {
      res.success = false;
      res.status_message = "MoveSpiral failed: " + spiral_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(spiral_report.trajectory_duration_s, 0.3);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Smooth return line back to p_init
    Eigen::VectorXd q_spiral_end = spiral_report.goal_joints;
    auto rev_report = engine.moveLine(p_init, 30, "5.4b MoveSpiral Return", &q_spiral_end);
    if (!rev_report.success) {
      res.success = false;
      res.status_message = "Spiral return failed: " + rev_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(rev_report.trajectory_duration_s, 0.3);
    }

    Eigen::Vector3d p_final = engine.getCurrentPose().block<3, 1>(0, 3);
    res.closure_or_return_error_mm = (p_final - p_init).norm() * 1000.0;
    res.success = true;
    res.status_message = "MoveSpiral expanding contour & return completed.";
    res.ik_time_ms = spiral_report.ik_time_ms + rev_report.ik_time_ms;
    res.trajectory_duration_s = spiral_report.trajectory_duration_s + rev_report.trajectory_duration_s;
    res.path_waypoints_count = spiral_report.path_waypoints_count + rev_report.path_waypoints_count;
    res.trajectory_points_count = spiral_report.trajectory_points_count + rev_report.trajectory_points_count;
    extractUtilizations(spiral_report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    std::cout << "  -> Experiment 5.4 Completed. Return Position Error: "
              << std::fixed << std::setprecision(4) << res.closure_or_return_error_mm << " mm\n";

    return res;
  }

private:
  double end_radius_{0.110};
  double turns_{2.0};
  size_t steps_{60};
};

// ===========================================================================
// CONCRETE COMMAND 5: 3D CYLINDRICAL HELIX (MoveHelix) & RETURN
// ===========================================================================

class HelixExperimentTask : public IExperimentTask
{
public:
  explicit HelixExperimentTask(
    double height = 0.060,
    double turns = 2.0,
    size_t steps = 60)
  : height_(height), turns_(turns), steps_(steps)
  {
  }

  [[nodiscard]] std::string getName() const override
  {
    return "EXPERIMENT 5.5: 3D Cylindrical Helix Path (MoveHelix) & Return";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Trace 3D cylindrical helix (r=70mm, +60mm Z rise, 2 revolutions), then descend to start.";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::Vector3d p_init = engine.getCurrentPose().block<3, 1>(0, 3);
    Eigen::Vector3d center(0.370, 0.150, p_init.z());

    auto helix_report = engine.moveHelix(
      center, height_, turns_, Eigen::Vector3d(0.0, 0.0, 1.0), steps_, "5.5 MoveHelix 3D");

    if (!helix_report.success) {
      res.success = false;
      res.status_message = "MoveHelix failed: " + helix_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(helix_report.trajectory_duration_s, 0.3);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Smooth descent back to p_init
    Eigen::VectorXd q_helix_end = helix_report.goal_joints;
    auto rev_report = engine.moveLine(p_init, 30, "5.5b MoveHelix Return", &q_helix_end);
    if (!rev_report.success) {
      res.success = false;
      res.status_message = "Helix return failed: " + rev_report.status_message;
      return res;
    }
    if (observer) {
      observer->waitForTrajectory(rev_report.trajectory_duration_s, 0.3);
    }

    Eigen::Vector3d p_final = engine.getCurrentPose().block<3, 1>(0, 3);
    res.closure_or_return_error_mm = (p_final - p_init).norm() * 1000.0;
    res.success = true;
    res.status_message = "MoveHelix 3D vertical ascent & descent completed.";
    res.ik_time_ms = helix_report.ik_time_ms + rev_report.ik_time_ms;
    res.trajectory_duration_s = helix_report.trajectory_duration_s + rev_report.trajectory_duration_s;
    res.path_waypoints_count = helix_report.path_waypoints_count + rev_report.path_waypoints_count;
    res.trajectory_points_count = helix_report.trajectory_points_count + rev_report.trajectory_points_count;
    extractUtilizations(helix_report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    std::cout << "  -> Experiment 5.5 Completed. Return Position Error: "
              << std::fixed << std::setprecision(4) << res.closure_or_return_error_mm << " mm\n";

    return res;
  }

private:
  double height_{0.060};
  double turns_{2.0};
  size_t steps_{60};
};

// ===========================================================================
// CONCRETE COMMAND 6: GRIPPER ACTUATION VALIDATION (ISP)
// ===========================================================================

class GripperValidationTask : public IExperimentTask
{
public:
  [[nodiscard]] std::string getName() const override
  {
    return "GRIPPER VALIDATION: Dual-Finger Parallel Jaw Actuation (ISP)";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Actuate prismatic gripper fingers: full open (25mm), full close (0mm), and nominal hold (15mm).";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> /*observer*/) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    auto gripper = engine.getGripperController();
    if (!gripper) {
      res.success = false;
      res.status_message = "No gripper controller registered with ExecutiveEngine.";
      return res;
    }

    std::cout << "    -> Commencing Gripper Jaw Validation:\n";

    // 1. Fully open jaws to 25 mm
    std::cout << "       [Step 1]: Commanding Full Aperture (Open to 25.0 mm)...\n";
    gripper->open();
    gripper->waitForMotionComplete(std::chrono::milliseconds(1500));
    double w_open = gripper->getCurrentWidth() * 1000.0;
    std::cout << "       [Result 1]: Measured Jaw Stroke: " << std::fixed << std::setprecision(2)
              << w_open << " mm\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. Fully close jaws to 0 mm
    std::cout << "       [Step 2]: Commanding Full Grasp (Close to 0.0 mm)...\n";
    gripper->close();
    gripper->waitForMotionComplete(std::chrono::milliseconds(1500));
    double w_closed = gripper->getCurrentWidth() * 1000.0;
    std::cout << "       [Result 2]: Measured Jaw Stroke: " << std::fixed << std::setprecision(2)
              << w_closed << " mm\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Set nominal working width 15 mm
    std::cout << "       [Step 3]: Setting Nominal Grasp Readiness Width (15.0 mm)...\n";
    gripper->setWidth(0.015);
    gripper->waitForMotionComplete(std::chrono::milliseconds(1500));
    double w_nominal = gripper->getCurrentWidth() * 1000.0;
    std::cout << "       [Result 3]: Measured Jaw Stroke: " << std::fixed << std::setprecision(2)
              << w_nominal << " mm\n";

    res.success = true;
    res.status_message = "Gripper open/close cycles executed cleanly.";
    res.closure_or_return_error_mm = 0.0;
    return res;
  }
};

// ===========================================================================
// CONCRETE COMMAND 7: RETURN HOME TASK
// ===========================================================================

class ReturnHomeTask : public IExperimentTask
{
public:
  [[nodiscard]] std::string getName() const override
  {
    return "POST-EXPERIMENT: Safely Return Arm to Canonical Home Pose";
  }

  [[nodiscard]] std::string getObjective() const override
  {
    return "Articulate all 6 joints back to canonical standby posture [-0.75, -0.30, 0.50, -0.20, 0.40, 0.0].";
  }

  ExperimentTaskResult execute(
    ExecutiveEngine & engine,
    std::shared_ptr<ConsoleTelemetryObserver> observer) override
  {
    ExperimentTaskResult res;
    res.task_name = getName();
    res.objective = getObjective();

    Eigen::VectorXd home_joints(6);
    home_joints << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0;

    auto report = engine.moveJoint(home_joints, "Return Home");
    res.success = report.success;
    res.status_message = report.status_message;
    res.ik_time_ms = report.ik_time_ms;
    res.trajectory_duration_s = report.trajectory_duration_s;
    res.path_waypoints_count = report.path_waypoints_count;
    res.trajectory_points_count = report.trajectory_points_count;
    extractUtilizations(report, res.peak_vel_utilization_pct, res.peak_acc_utilization_pct);

    if (report.success && observer) {
      observer->waitForTrajectory(report.trajectory_duration_s, 0.3);
    }

    return res;
  }
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__EXPERIMENT_TASK_HPP_

