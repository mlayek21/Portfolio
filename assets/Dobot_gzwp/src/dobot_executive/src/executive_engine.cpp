#include "dobot_executive/executive_engine.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>

namespace dobot_executive
{

// ===========================================================================
// ConsoleTelemetryObserver Implementation
// ===========================================================================
void ConsoleTelemetryObserver::onExecutionComplete(
  const std::string & /*task_name*/,
  const TaskExecutionReport & report)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!report.success) {
    is_tracking_ = false;
  }
}

// ===========================================================================
// ExecutiveEngine Constructors (Dependency Inversion Principle)
// ===========================================================================

ExecutiveEngine::ExecutiveEngine(
  rclcpp::Node::SharedPtr node,
  ExecutiveConfig config)
: ExecutiveEngine(
    std::move(node),
    std::move(config),
    std::make_shared<dobot_kinematics::GeneralizedKinematicsEngine>(),
    nullptr,
    nullptr,
    nullptr)
{
}

ExecutiveEngine::ExecutiveEngine(
  rclcpp::Node::SharedPtr node,
  ExecutiveConfig config,
  std::shared_ptr<dobot_kinematics::GeneralizedKinematicsEngine> kinematics,
  std::shared_ptr<dobot_motion_planning::MotionPlanningEngine> planner,
  std::shared_ptr<dobot_trajectory::TrajectoryPlanningEngine> trajectory_engine,
  std::shared_ptr<IGripperController> gripper)
: node_(std::move(node)),
  config_(std::move(config)),
  kinematics_(kinematics ? std::move(kinematics) : std::make_shared<dobot_kinematics::GeneralizedKinematicsEngine>()),
  planner_(planner ? std::move(planner) : std::make_shared<dobot_motion_planning::MotionPlanningEngine>(kinematics_)),
  trajectory_engine_(trajectory_engine ? std::move(trajectory_engine) : std::make_shared<dobot_trajectory::TrajectoryPlanningEngine>()),
  gripper_(std::move(gripper))
{
  // 1. Configure internal subsystems
  planner_->setPlanner(config_.planner_type);
  planner_->setOptimizer(config_.optimizer_type);

  trajectory_engine_->setInterpolator(config_.interpolator_type);
  trajectory_engine_->setProfiler(config_.profiler_type);

  // 2. Setup ROS 2 Subscribers and Publishers
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_states_topic,
    10,
    std::bind(&ExecutiveEngine::jointStateCallback, this, std::placeholders::_1));

  trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    config_.trajectory_topic,
    10);

  RCLCPP_INFO(
    node_->get_logger(),
    "ExecutiveEngine initialized. Listening to: %s, Publishing trajectories on: %s",
    config_.joint_states_topic.c_str(),
    config_.trajectory_topic.c_str());
}

// ---------------------------------------------------------------------------
// Configuration & Environment Setup (Fluent Interface)
// ---------------------------------------------------------------------------

ExecutiveEngine & ExecutiveEngine::setPlanner(dobot_motion_planning::PlannerType type)
{
  config_.planner_type = type;
  planner_->setPlanner(type);
  return *this;
}

ExecutiveEngine & ExecutiveEngine::setOptimizer(dobot_motion_planning::OptimizerType type)
{
  config_.optimizer_type = type;
  planner_->setOptimizer(type);
  return *this;
}

ExecutiveEngine & ExecutiveEngine::setInterpolator(dobot_trajectory::InterpolatorType type)
{
  config_.interpolator_type = type;
  trajectory_engine_->setInterpolator(type);
  return *this;
}

ExecutiveEngine & ExecutiveEngine::setProfiler(dobot_trajectory::ProfilerType type)
{
  config_.profiler_type = type;
  trajectory_engine_->setProfiler(type);
  return *this;
}

ExecutiveEngine & ExecutiveEngine::addObstacle(
  std::shared_ptr<dobot_motion_planning::IObstacle> obstacle)
{
  planner_->addObstacle(std::move(obstacle));
  return *this;
}

ExecutiveEngine & ExecutiveEngine::clearObstacles()
{
  planner_->clearObstacles();
  return *this;
}

// ---------------------------------------------------------------------------
// Gripper Controller & TCP Configuration
// ---------------------------------------------------------------------------

ExecutiveEngine & ExecutiveEngine::setGripperController(std::shared_ptr<IGripperController> gripper)
{
  gripper_ = std::move(gripper);
  return *this;
}

std::shared_ptr<IGripperController> ExecutiveEngine::getGripperController() const noexcept
{
  return gripper_;
}

bool ExecutiveEngine::hasGripper() const noexcept
{
  return (gripper_ != nullptr);
}

ExecutiveEngine & ExecutiveEngine::setTCPOffset(const Eigen::Matrix4d & tcp_offset)
{
  config_.tcp_offset = tcp_offset;
  return *this;
}

const Eigen::Matrix4d & ExecutiveEngine::getTCPOffset() const noexcept
{
  return config_.tcp_offset;
}

Eigen::Matrix4d ExecutiveEngine::getCurrentTCPPose() const
{
  return getCurrentPose() * config_.tcp_offset;
}

// ---------------------------------------------------------------------------
// Observer Pattern Management
// ---------------------------------------------------------------------------

void ExecutiveEngine::addObserver(std::shared_ptr<ITelemetryObserver> observer)
{
  if (!observer) return;
  std::lock_guard<std::mutex> lock(observers_mutex_);
  observers_.push_back(std::move(observer));
}

void ExecutiveEngine::removeObserver(std::shared_ptr<ITelemetryObserver> observer)
{
  std::lock_guard<std::mutex> lock(observers_mutex_);
  observers_.erase(
    std::remove(observers_.begin(), observers_.end(), observer),
    observers_.end());
}

void ExecutiveEngine::notifyJointStateUpdated(
  const Eigen::VectorXd & joints,
  const Eigen::Matrix4d & tcp_pose)
{
  std::lock_guard<std::mutex> lock(observers_mutex_);
  for (const auto & obs : observers_) {
    if (obs) obs->onJointStateUpdated(joints, tcp_pose);
  }
}

void ExecutiveEngine::notifyTrajectoryDispatched(
  const std::string & task_name,
  double duration,
  size_t count)
{
  std::lock_guard<std::mutex> lock(observers_mutex_);
  for (const auto & obs : observers_) {
    if (obs) obs->onTrajectoryDispatched(task_name, duration, count);
  }
}

void ExecutiveEngine::notifyExecutionComplete(
  const std::string & task_name,
  const TaskExecutionReport & report)
{
  std::lock_guard<std::mutex> lock(observers_mutex_);
  for (const auto & obs : observers_) {
    if (obs) obs->onExecutionComplete(task_name, report);
  }
}

void ExecutiveEngine::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  Eigen::VectorXd q_copy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto & target_names = trajectory_engine_->getJointNames();

    for (size_t i = 0; i < target_names.size(); ++i) {
      for (size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == target_names[i] && j < msg->position.size()) {
          current_joints_[i] = msg->position[j];
          break;
        }
      }
    }
    joint_state_received_ = true;
    q_copy = current_joints_;
  }

  // Reactive Observer callback outside state lock to avoid deadlocks
  Eigen::Matrix4d T_tcp = kinematics_->computeFK(q_copy) * config_.tcp_offset;
  notifyJointStateUpdated(q_copy, T_tcp);
}

Eigen::VectorXd ExecutiveEngine::getCurrentJoints() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_joints_;
}

Eigen::Matrix4d ExecutiveEngine::getCurrentPose() const
{
  return kinematics_->computeFK(getCurrentJoints());
}

bool ExecutiveEngine::hasReceivedJointStates() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_state_received_;
}

bool ExecutiveEngine::executeTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::string & task_name)
{
  if (trajectory.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "ExecutiveEngine::executeTrajectory: Empty trajectory!");
    return false;
  }
  double duration = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
  notifyTrajectoryDispatched(task_name, duration, trajectory.points.size());

  trajectory_pub_->publish(trajectory);
  RCLCPP_INFO(
    node_->get_logger(),
    "Dispatched trajectory [%s] with %zu points (Duration: %.3f s) to %s",
    task_name.c_str(),
    trajectory.points.size(),
    duration,
    config_.trajectory_topic.c_str());
  return true;
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planCartesian(
  const Eigen::Vector3d & target_pos,
  const Eigen::Matrix3d & target_rot,
  const Eigen::VectorXd * seed_q)
{
  TaskExecutionReport report;
  report.target_position = target_pos;

  // 1. Solve Inverse Kinematics
  Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
  T_target.block<3, 3>(0, 0) = target_rot;
  T_target.block<3, 1>(0, 3) = target_pos;

  const auto ik_start_t = std::chrono::high_resolution_clock::now();
  std::optional<dobot_kinematics::IKSolution> ik_result;

  if (seed_q) {
    ik_result = kinematics_->computeIK(T_target, *seed_q);
  } else {
    // 1. Primary attempt: solve from current joint configuration (minimal movement)
    ik_result = kinematics_->computeIK(T_target, getCurrentJoints());

    // 2. Fallback attempt: if trapped in local minimum, solve from canonical home posture
    if (!ik_result.has_value() || !ik_result->converged) {
      Eigen::VectorXd home_seed(6);
      home_seed << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0;
      ik_result = kinematics_->computeIK(T_target, home_seed);
    }
  }

  const auto ik_end_t = std::chrono::high_resolution_clock::now();

  report.ik_time_ms = std::chrono::duration<double, std::milli>(ik_end_t - ik_start_t).count();

  if (!ik_result.has_value() || !ik_result->converged) {
    report.success = false;
    report.status_message =
      "Inverse Kinematics failed to converge. Target pose outside reachable workspace or singular.";
    return {report, trajectory_msgs::msg::JointTrajectory()};
  }

  // 2. Plan trajectory to the IK solved joint solution
  auto [joint_report, traj] = planJoint(ik_result->joint_angles);
  joint_report.ik_time_ms = report.ik_time_ms;
  joint_report.target_position = target_pos;

  return {joint_report, traj};
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planJoint(const Eigen::VectorXd & target_joints)
{
  TaskExecutionReport report;
  const Eigen::VectorXd start_q = getCurrentJoints();
  report.start_joints = start_q;
  report.goal_joints = target_joints;

  // 1. Motion Planning (Collision Avoidance via RRT* / RRT)
  dobot_motion_planning::PlannerConfig plan_cfg;
  plan_cfg.step_size = config_.planning_step_size;
  plan_cfg.goal_bias = config_.goal_bias;
  plan_cfg.rrt_star_radius = config_.rrt_star_radius;
  plan_cfg.timeout_seconds = config_.planning_timeout;

  const auto plan_start_t = std::chrono::high_resolution_clock::now();
  const auto plan_result = planner_->plan(start_q, target_joints, plan_cfg);
  const auto plan_end_t = std::chrono::high_resolution_clock::now();

  report.planning_time_ms =
    std::chrono::duration<double, std::milli>(plan_end_t - plan_start_t).count();

  if (!plan_result.success) {
    report.success = false;
    report.status_message = "Motion Planning failed: " + plan_result.status_message;
    return {report, trajectory_msgs::msg::JointTrajectory()};
  }

  report.path_waypoints_count = plan_result.path.size();

  // 2. Trajectory Parameterization (B-Splines & Velocity Profiling)
  const auto traj_result = trajectory_engine_->plan(
    plan_result.path, config_.sample_dt, config_.frame_id);

  if (!traj_result.success) {
    report.success = false;
    report.status_message = "Trajectory Generation failed: " + traj_result.status_message;
    return {report, trajectory_msgs::msg::JointTrajectory()};
  }

  report.success = true;
  report.status_message = "Trajectory successfully synthesized.";
  report.trajectory_duration_s = traj_result.duration;
  report.trajectory_points_count = traj_result.trajectory.points.size();
  report.utilization = traj_result.report;

  return {report, traj_result.trajectory};
}

TaskExecutionReport ExecutiveEngine::moveCartesian(
  const Eigen::Vector3d & target_pos,
  const Eigen::Matrix3d & target_rot,
  const Eigen::VectorXd * seed_q)
{
  auto [report, traj] = planCartesian(target_pos, target_rot, seed_q);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, "MoveCartesian");
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete("MoveCartesian", report);
  return report;
}

TaskExecutionReport ExecutiveEngine::moveJoint(
  const Eigen::VectorXd & target_joints,
  const std::string & task_name)
{
  auto [report, traj] = planJoint(target_joints);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

// ---------------------------------------------------------------------------
// High-Level Cartesian Geometric Path Planning
// ---------------------------------------------------------------------------

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planCartesianPath(
  const ICartesianPath & path,
  size_t num_samples,
  const Eigen::VectorXd * seed_q)
{
  std::vector<Eigen::Matrix4d> waypoints = path.sample(num_samples);
  return planCartesianWaypoints(waypoints, seed_q);
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planCartesianWaypoints(
  const std::vector<Eigen::Matrix4d> & waypoints,
  const Eigen::VectorXd * seed_q)
{
  TaskExecutionReport report;
  if (waypoints.size() < 2) {
    report.success = false;
    report.status_message = "At least 2 Cartesian waypoints required for path planning.";
    return {report, trajectory_msgs::msg::JointTrajectory()};
  }

  report.start_joints = getCurrentJoints();
  report.path_waypoints_count = waypoints.size();
  report.target_position = waypoints.back().block<3, 1>(0, 3);

  std::vector<Eigen::VectorXd> joint_path;
  joint_path.reserve(waypoints.size());

  Eigen::VectorXd q_seed = seed_q ? *seed_q : report.start_joints;
  double total_ik_ms = 0.0;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto ik_start = std::chrono::high_resolution_clock::now();
    auto ik_sol = kinematics_->computeIK(waypoints[i], q_seed, 1e-4, 1e-3, 200);

    // Multi-tier robust recovery for edge waypoints / continuous transitions
    if (!ik_sol.has_value() || !ik_sol->converged) {
      ik_sol = kinematics_->computeIK(waypoints[i], q_seed, 2e-4, 4e-3, 350);
    }
    if (!ik_sol.has_value() || !ik_sol->converged) {
      if (i > 0 && !joint_path.empty()) {
        ik_sol = kinematics_->computeIK(waypoints[i], joint_path.back(), 2e-4, 4e-3, 350);
      }
    }
    if (!ik_sol.has_value() || !ik_sol->converged) {
      ik_sol = kinematics_->computeIK(waypoints[i], report.start_joints, 2e-4, 5e-3, 350);
    }

    const auto ik_end = std::chrono::high_resolution_clock::now();
    total_ik_ms += std::chrono::duration<double, std::milli>(ik_end - ik_start).count();

    if (!ik_sol.has_value() || !ik_sol->converged) {
      report.success = false;
      report.status_message =
        "Cartesian IK tracking failed at waypoint " + std::to_string(i) +
        "/" + std::to_string(waypoints.size()) + ". Pose unreachable or in singularity.";
      return {report, trajectory_msgs::msg::JointTrajectory()};
    }

    joint_path.push_back(ik_sol->joint_angles);
    q_seed = ik_sol->joint_angles;  // Continuous seed chain ensures 0 flips
  }

  report.ik_time_ms = total_ik_ms;
  report.goal_joints = joint_path.back();

  // Trajectory Parameterization: B-Spline interpolation & velocity profiling
  const auto traj_result = trajectory_engine_->plan(
    joint_path, config_.sample_dt, config_.frame_id);

  if (!traj_result.success) {
    report.success = false;
    report.status_message = "Trajectory Generation failed: " + traj_result.status_message;
    return {report, trajectory_msgs::msg::JointTrajectory()};
  }

  report.success = true;
  report.status_message = "Continuous Cartesian path successfully synthesized.";
  report.trajectory_duration_s = traj_result.duration;
  report.trajectory_points_count = traj_result.trajectory.points.size();
  report.utilization = traj_result.report;

  return {report, traj_result.trajectory};
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planLine(
  const Eigen::Vector3d & target_pos,
  size_t steps,
  const Eigen::VectorXd * seed_q)
{
  Eigen::Matrix4d T_curr = getCurrentPose();
  Eigen::Vector3d p_start = T_curr.block<3, 1>(0, 3);
  Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

  CartesianLinePath line(p_start, target_pos, R_curr);
  return planCartesianPath(line, steps, seed_q);
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planCircle(
  const Eigen::Vector3d & center,
  double /*radius*/,
  const Eigen::Vector3d & plane_normal,
  double sweep_angle,
  size_t steps)
{
  Eigen::Matrix4d T_curr = getCurrentPose();
  Eigen::Vector3d p_start = T_curr.block<3, 1>(0, 3);
  Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

  CartesianCirclePath circle(
    center, p_start, plane_normal, R_curr, sweep_angle);
  return planCartesianPath(circle, steps);
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planRectangle(
  double width,
  double height,
  const Eigen::Vector3d & width_dir,
  const Eigen::Vector3d & height_dir,
  size_t steps_per_side)
{
  Eigen::Matrix4d T_curr = getCurrentPose();
  Eigen::Vector3d p_start = T_curr.block<3, 1>(0, 3);
  Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

  CartesianRectanglePath rect(
    p_start, width, height, width_dir, height_dir, R_curr);
  return planCartesianPath(rect, steps_per_side * 4 + 1);
}

TaskExecutionReport ExecutiveEngine::moveCartesianPath(
  const ICartesianPath & path,
  size_t num_samples,
  const Eigen::VectorXd * seed_q,
  const std::string & task_name)
{
  auto [report, traj] = planCartesianPath(path, num_samples, seed_q);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

TaskExecutionReport ExecutiveEngine::moveLine(
  const Eigen::Vector3d & target_pos,
  size_t steps,
  const std::string & task_name,
  const Eigen::VectorXd * seed_q)
{
  auto [report, traj] = planLine(target_pos, steps, seed_q);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

TaskExecutionReport ExecutiveEngine::moveCircle(
  const Eigen::Vector3d & center,
  double radius,
  const Eigen::Vector3d & plane_normal,
  double sweep_angle,
  size_t steps,
  const std::string & task_name)
{
  auto [report, traj] = planCircle(center, radius, plane_normal, sweep_angle, steps);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

TaskExecutionReport ExecutiveEngine::moveRectangle(
  double width,
  double height,
  const Eigen::Vector3d & width_dir,
  const Eigen::Vector3d & height_dir,
  size_t steps_per_side,
  const std::string & task_name)
{
  auto [report, traj] = planRectangle(width, height, width_dir, height_dir, steps_per_side);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planSpiral(
  const Eigen::Vector3d & center,
  double end_radius,
  double axial_height,
  double num_turns,
  const Eigen::Vector3d & axial_dir,
  size_t steps)
{
  Eigen::Matrix4d T_curr = getCurrentPose();
  Eigen::Vector3d p_start = T_curr.block<3, 1>(0, 3);
  Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

  CartesianSpiralPath spiral(
    center, p_start, end_radius, axial_height, num_turns, R_curr, axial_dir);
  return planCartesianPath(spiral, steps);
}

std::pair<TaskExecutionReport, trajectory_msgs::msg::JointTrajectory>
ExecutiveEngine::planHelix(
  const Eigen::Vector3d & center,
  double total_height,
  double num_turns,
  const Eigen::Vector3d & axial_dir,
  size_t steps)
{
  Eigen::Matrix4d T_curr = getCurrentPose();
  Eigen::Vector3d p_start = T_curr.block<3, 1>(0, 3);
  Eigen::Matrix3d R_curr = T_curr.block<3, 3>(0, 0);

  CartesianHelixPath helix(
    center, p_start, total_height, num_turns, R_curr, axial_dir);
  return planCartesianPath(helix, steps);
}

TaskExecutionReport ExecutiveEngine::moveSpiral(
  const Eigen::Vector3d & center,
  double end_radius,
  double axial_height,
  double num_turns,
  const Eigen::Vector3d & axial_dir,
  size_t steps,
  const std::string & task_name)
{
  auto [report, traj] = planSpiral(center, end_radius, axial_height, num_turns, axial_dir, steps);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

TaskExecutionReport ExecutiveEngine::moveHelix(
  const Eigen::Vector3d & center,
  double total_height,
  double num_turns,
  const Eigen::Vector3d & axial_dir,
  size_t steps,
  const std::string & task_name)
{
  auto [report, traj] = planHelix(center, total_height, num_turns, axial_dir, steps);
  if (report.success && config_.auto_execute) {
    executeTrajectory(traj, task_name);
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints_ = report.goal_joints;
  }
  notifyExecutionComplete(task_name, report);
  return report;
}

}  // namespace dobot_executive
