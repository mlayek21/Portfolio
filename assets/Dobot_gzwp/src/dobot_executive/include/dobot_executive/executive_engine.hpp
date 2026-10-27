#ifndef DOBOT_EXECUTIVE__EXECUTIVE_ENGINE_HPP_
#define DOBOT_EXECUTIVE__EXECUTIVE_ENGINE_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "dobot_kinematics/kinematics_engine.hpp"
#include "dobot_motion_planning/motion_planning_engine.hpp"
#include "dobot_trajectory/trajectory_planning_engine.hpp"
#include "dobot_executive/cartesian_path.hpp"
#include "dobot_executive/gripper_controller.hpp"
#include "dobot_executive/telemetry_observer.hpp"

namespace dobot_executive
{

// ===========================================================================
// 1. DATA MODELS & TELEMETRY (SRP)
// ===========================================================================

struct ExecutiveConfig
{
  std::string frame_id{"base_link"};
  std::string trajectory_topic{"/dobot_arm_controller/joint_trajectory"};
  std::string joint_states_topic{"/joint_states"};

  dobot_motion_planning::PlannerType planner_type{
    dobot_motion_planning::PlannerType::RRT_STAR};
  dobot_motion_planning::OptimizerType optimizer_type{
    dobot_motion_planning::OptimizerType::SHORTCUT};

  dobot_trajectory::InterpolatorType interpolator_type{
    dobot_trajectory::InterpolatorType::CUBIC_BSPLINE};
  dobot_trajectory::ProfilerType profiler_type{
    dobot_trajectory::ProfilerType::TRAPEZOIDAL};

  double planning_step_size{0.08};      ///< rad
  double goal_bias{0.20};
  double rrt_star_radius{0.50};
  double planning_timeout{5.0};         ///< seconds
  double sample_dt{0.02};               ///< 50 Hz trajectory sampling
  bool auto_execute{true};

  // Tool Center Point (TCP) transformation from Link6 flange frame
  // Default is +112mm along tool Z-axis for the 2-finger parallel gripper
  Eigen::Matrix4d tcp_offset{
    []() {
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T(2, 3) = 0.112;  // +112 mm along tool Z
      return T;
    }()
  };
};

struct TaskExecutionReport
{
  bool success{false};
  std::string status_message;
  double ik_time_ms{0.0};
  double planning_time_ms{0.0};
  double trajectory_duration_s{0.0};
  size_t path_waypoints_count{0};
  size_t trajectory_points_count{0};
  Eigen::VectorXd start_joints;
  Eigen::VectorXd goal_joints;
  Eigen::Vector3d target_position;
  dobot_trajectory::TrajectoryUtilizationReport utilization;
};

// ===========================================================================
// 2. UNIFIED MASTER ORCHESTRATION ENGINE: ExecutiveEngine
// ===========================================================================

class ExecutiveEngine
{
public:
  /**
   * @brief Construct ExecutiveEngine bound to an active ROS 2 node.
   */
  explicit ExecutiveEngine(
    rclcpp::Node::SharedPtr node,
    ExecutiveConfig config = ExecutiveConfig());

  /**
   * @brief Construct ExecutiveEngine with Dependency Injection (DIP).
   */
  ExecutiveEngine(
    rclcpp::Node::SharedPtr node,
    ExecutiveConfig config,
    std::shared_ptr<dobot_kinematics::GeneralizedKinematicsEngine> kinematics,
    std::shared_ptr<dobot_motion_planning::MotionPlanningEngine> planner = nullptr,
    std::shared_ptr<dobot_trajectory::TrajectoryPlanningEngine> trajectory_engine = nullptr,
    std::shared_ptr<IGripperController> gripper = nullptr);

  // -------------------------------------------------------------------------
  // Configuration & Environment Setup (Fluent Interface)
  // -------------------------------------------------------------------------
  ExecutiveEngine & setPlanner(dobot_motion_planning::PlannerType type);
  ExecutiveEngine & setOptimizer(dobot_motion_planning::OptimizerType type);
  ExecutiveEngine & setInterpolator(dobot_trajectory::InterpolatorType type);
  ExecutiveEngine & setProfiler(dobot_trajectory::ProfilerType type);

  ExecutiveEngine & addObstacle(std::shared_ptr<dobot_motion_planning::IObstacle> obstacle);
  ExecutiveEngine & clearObstacles();

  // -------------------------------------------------------------------------
  // Gripper Controller (ISP) & Tool Center Point (TCP) Configuration
  // -------------------------------------------------------------------------
  ExecutiveEngine & setGripperController(std::shared_ptr<IGripperController> gripper);
  [[nodiscard]] std::shared_ptr<IGripperController> getGripperController() const noexcept;
  [[nodiscard]] bool hasGripper() const noexcept;

  ExecutiveEngine & setTCPOffset(const Eigen::Matrix4d & tcp_offset);
  [[nodiscard]] const Eigen::Matrix4d & getTCPOffset() const noexcept;
  [[nodiscard]] Eigen::Matrix4d getCurrentTCPPose() const;

  // -------------------------------------------------------------------------
  // Observer Pattern Management
  // -------------------------------------------------------------------------
  void addObserver(std::shared_ptr<ITelemetryObserver> observer);
  void removeObserver(std::shared_ptr<ITelemetryObserver> observer);

  // -------------------------------------------------------------------------
  // State Inspection
  // -------------------------------------------------------------------------
  [[nodiscard]] Eigen::VectorXd getCurrentJoints() const;
  [[nodiscard]] Eigen::Matrix4d getCurrentPose() const;  ///< Returns flange pose T_0^flange
  [[nodiscard]] bool hasReceivedJointStates() const;

  [[nodiscard]] const dobot_kinematics::GeneralizedKinematicsEngine & getKinematics() const noexcept
  {
    return *kinematics_;
  }
  [[nodiscard]] const dobot_motion_planning::MotionPlanningEngine & getPlanner() const noexcept
  {
    return *planner_;
  }
  [[nodiscard]] const dobot_trajectory::TrajectoryPlanningEngine & getTrajectoryEngine() const noexcept
  {
    return *trajectory_engine_;
  }

  // -------------------------------------------------------------------------
  // End-to-End Orchestrated Planning & Execution API
  // -------------------------------------------------------------------------

  /**
   * @brief High-level Cartesian Goal Planning:
   * Cartesian Pose -> IK (Singularity-Damped) -> RRT* (Obstacle Avoidance) -> B-Spline -> Trajectory
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planCartesian(
    const Eigen::Vector3d & target_pos,
    const Eigen::Matrix3d & target_rot,
    const Eigen::VectorXd * seed_q = nullptr);

  /**
   * @brief Joint Space Goal Planning:
   * Joint Target -> RRT* (Obstacle Avoidance) -> B-Spline -> Trajectory
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planJoint(
    const Eigen::VectorXd & target_joints);

  /**
   * @brief Execute a pre-planned JointTrajectory directly to the robot controller.
   */
  bool executeTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::string & task_name = "Trajectory");

  /**
   * @brief Convenience single-call plan and dispatch for Cartesian goal.
   */
  TaskExecutionReport moveCartesian(
    const Eigen::Vector3d & target_pos,
    const Eigen::Matrix3d & target_rot,
    const Eigen::VectorXd * seed_q = nullptr);

  /**
   * @brief Convenience single-call plan and dispatch for Joint goal.
   */
  TaskExecutionReport moveJoint(
    const Eigen::VectorXd & target_joints,
    const std::string & task_name = "Joint Motion");

  // -------------------------------------------------------------------------
  // High-Level Cartesian Geometric Path Planning (MoveL, MoveC, MoveRect)
  // -------------------------------------------------------------------------

  /**
   * @brief Plan a continuous Cartesian trajectory from an abstract ICartesianPath strategy.
   * Discretizes the path, solves continuous seeded IK, fits B-Splines, and profiles time.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planCartesianPath(
    const ICartesianPath & path,
    size_t num_samples = 30,
    const Eigen::VectorXd * seed_q = nullptr);

  /**
   * @brief Plan trajectory through an explicit sequence of SE(3) Cartesian waypoints.
   * Continuous IK solver guarantees zero joint flips along the path.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planCartesianWaypoints(
    const std::vector<Eigen::Matrix4d> & waypoints,
    const Eigen::VectorXd * seed_q = nullptr);

  /**
   * @brief Experiment 5.1 / MoveL: Plan a straight line in Cartesian space from current pose to target_pos.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planLine(
    const Eigen::Vector3d & target_pos,
    size_t steps = 25,
    const Eigen::VectorXd * seed_q = nullptr);

  /**
   * @brief Experiment 5.2 / MoveC: Plan a circular arc / circle in Cartesian space.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planCircle(
    const Eigen::Vector3d & center,
    double radius,
    const Eigen::Vector3d & plane_normal = Eigen::Vector3d(0.0, 0.0, 1.0),
    double sweep_angle = 2.0 * M_PI,
    size_t steps = 36);

  /**
   * @brief Experiment 5.3 / MoveRect: Plan a closed rectangular perimeter in Cartesian space.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planRectangle(
    double width,
    double height,
    const Eigen::Vector3d & width_dir = Eigen::Vector3d(1.0, 0.0, 0.0),
    const Eigen::Vector3d & height_dir = Eigen::Vector3d(0.0, 1.0, 0.0),
    size_t steps_per_side = 10);

  /**
   * @brief Convenience single-call plan and dispatch methods for Cartesian paths
   */
  TaskExecutionReport moveCartesianPath(
    const ICartesianPath & path,
    size_t num_samples = 30,
    const Eigen::VectorXd * seed_q = nullptr,
    const std::string & task_name = "Cartesian Path");

  TaskExecutionReport moveLine(
    const Eigen::Vector3d & target_pos,
    size_t steps = 25,
    const std::string & task_name = "MoveL",
    const Eigen::VectorXd * seed_q = nullptr);

  TaskExecutionReport moveCircle(
    const Eigen::Vector3d & center,
    double radius,
    const Eigen::Vector3d & plane_normal = Eigen::Vector3d(0.0, 0.0, 1.0),
    double sweep_angle = 2.0 * M_PI,
    size_t steps = 36,
    const std::string & task_name = "MoveC");

  TaskExecutionReport moveRectangle(
    double width,
    double height,
    const Eigen::Vector3d & width_dir = Eigen::Vector3d(1.0, 0.0, 0.0),
    const Eigen::Vector3d & height_dir = Eigen::Vector3d(0.0, 1.0, 0.0),
    size_t steps_per_side = 10,
    const std::string & task_name = "MoveRect");

  /**
   * @brief Experiment 5.4 / MoveSpiral: Plan a 3D Conical / Archimedean spiral.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planSpiral(
    const Eigen::Vector3d & center,
    double end_radius,
    double axial_height,
    double num_turns = 2.0,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0),
    size_t steps = 50);

  /**
   * @brief Experiment 5.5 / MoveHelix: Plan a 3D Cylindrical helix.
   */
  [[nodiscard]] std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory> planHelix(
    const Eigen::Vector3d & center,
    double total_height,
    double num_turns = 2.0,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0),
    size_t steps = 50);

  TaskExecutionReport moveSpiral(
    const Eigen::Vector3d & center,
    double end_radius,
    double axial_height,
    double num_turns = 2.0,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0),
    size_t steps = 50,
    const std::string & task_name = "MoveSpiral");

  TaskExecutionReport moveHelix(
    const Eigen::Vector3d & center,
    double total_height,
    double num_turns = 2.0,
    const Eigen::Vector3d & axial_dir = Eigen::Vector3d(0.0, 0.0, 1.0),
    size_t steps = 50,
    const std::string & task_name = "MoveHelix");

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  void notifyJointStateUpdated(const Eigen::VectorXd & joints, const Eigen::Matrix4d & tcp_pose);
  void notifyTrajectoryDispatched(const std::string & task_name, double duration, size_t count);
  void notifyExecutionComplete(const std::string & task_name, const TaskExecutionReport & report);

  rclcpp::Node::SharedPtr node_;
  ExecutiveConfig config_;

  // Engine Subsystems (DIP)
  std::shared_ptr<dobot_kinematics::GeneralizedKinematicsEngine> kinematics_;
  std::shared_ptr<dobot_motion_planning::MotionPlanningEngine> planner_;
  std::shared_ptr<dobot_trajectory::TrajectoryPlanningEngine> trajectory_engine_;
  std::shared_ptr<IGripperController> gripper_;

  // Observer Pattern Observers
  std::vector<std::shared_ptr<ITelemetryObserver>> observers_;
  mutable std::mutex observers_mutex_;

  // ROS 2 Communications
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  mutable std::mutex state_mutex_;
  Eigen::VectorXd current_joints_{
    (Eigen::VectorXd(6) << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0).finished()};
  bool joint_state_received_{false};
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__EXECUTIVE_ENGINE_HPP_
