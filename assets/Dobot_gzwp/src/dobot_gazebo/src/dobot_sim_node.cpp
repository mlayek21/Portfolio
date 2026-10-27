#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "dobot_kinematics/kinematics_engine.hpp"

namespace dobot_gazebo
{

class DobotSimNode : public rclcpp::Node
{
public:
  DobotSimNode()
  : Node("dobot_sim_node"),
    joint_names_{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
      "left_finger_joint", "right_finger_joint"
    },
    current_positions_{-0.75, -0.30, 0.50, -0.20, 0.40, 0.0},
    current_velocities_(6, 0.0),
    gripper_position_(0.0),
    gripper_target_(0.0),
    has_active_goal_(false)
  {
    // 1. Core Publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/dobot/end_effector_pose", 10);

    trace_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/dobot/end_effector_trace", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/dobot/end_effector_path", 10);

    // 2. Trajectory Subscriber
    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/dobot_arm_controller/joint_trajectory",
      10,
      std::bind(&DobotSimNode::trajectoryCallback, this, std::placeholders::_1));

    // 3. Gripper Command Subscriber
    gripper_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/dobot/gripper/command",
      10,
      std::bind(&DobotSimNode::gripperCallback, this, std::placeholders::_1));

    // 4. 50 Hz Simulation & Tracing Loop (20 ms dt)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DobotSimNode::simStep, this));

    RCLCPP_INFO(this->get_logger(), "Dobot Nova 5 Synchronized Simulator & Real-time Tracer initialized at 50 Hz.");
    RCLCPP_INFO(this->get_logger(), "  - Trajectory Topic:      /dobot_arm_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  - Gripper Topic:         /dobot/gripper/command (0.0=closed to 0.025=open)");
    RCLCPP_INFO(this->get_logger(), "  - Joint States Topic:    /joint_states");
    RCLCPP_INFO(this->get_logger(), "  - End-Effector Pose:     /dobot/end_effector_pose");
    RCLCPP_INFO(this->get_logger(), "  - Visual 3D Trace Trail: /dobot/end_effector_trace");
    RCLCPP_INFO(this->get_logger(), "  - Nav Path Topic:        /dobot/end_effector_path");
  }

private:
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty JointTrajectory message!");
      return;
    }

    trajectory_queue_ = msg->points;
    traj_start_time_ = this->now();
    start_positions_ = current_positions_;
    has_active_goal_ = true;

    // Clear trace ribbon for new trajectory motion
    trace_points_.clear();
    trace_poses_.clear();

    const double total_duration =
      rclcpp::Duration(trajectory_queue_.back().time_from_start).seconds();

    RCLCPP_INFO(
      this->get_logger(),
      "Received new trajectory with %zu waypoints (Planned Duration: %.3f s).",
      trajectory_queue_.size(),
      total_duration);
  }

  void gripperCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    gripper_target_ = std::clamp(msg->data, 0.0, 0.025);
    RCLCPP_INFO(
      this->get_logger(),
      "Received gripper command: target width = %.4f m (%s)",
      gripper_target_,
      (gripper_target_ < 0.005 ? "CLOSED" : (gripper_target_ > 0.020 ? "FULLY OPEN" : "PARTIAL")));
  }

  void simStep()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = this->now();

    // 1. Time-Parameterized Synchronized Multi-Joint Motion Playback
    if (has_active_goal_ && !trajectory_queue_.empty()) {
      double t_elapsed = (now - traj_start_time_).seconds();
      double t_final = rclcpp::Duration(trajectory_queue_.back().time_from_start).seconds();

      if (t_elapsed >= t_final) {
        // Final point reached
        const auto & last_pt = trajectory_queue_.back();
        for (size_t i = 0; i < 6 && i < last_pt.positions.size(); ++i) {
          current_positions_[i] = last_pt.positions[i];
          current_velocities_[i] = 0.0;
        }
        has_active_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "Trajectory execution finished at target configuration.");
      } else {
        // Find interval [k, k+1] matching current elapsed time
        size_t k = 0;
        while (k + 1 < trajectory_queue_.size() &&
               rclcpp::Duration(trajectory_queue_[k + 1].time_from_start).seconds() < t_elapsed)
        {
          k++;
        }

        const auto & p0 = trajectory_queue_[k];
        const auto & p1 = trajectory_queue_[std::min(k + 1, trajectory_queue_.size() - 1)];

        double t0 = rclcpp::Duration(p0.time_from_start).seconds();
        double t1 = rclcpp::Duration(p1.time_from_start).seconds();
        double dt_seg = t1 - t0;
        double alpha = (dt_seg > 1e-6) ? std::clamp((t_elapsed - t0) / dt_seg, 0.0, 1.0) : 0.0;

        // Synchronous interpolation across all 6 joints guarantees exact Cartesian fidelity
        for (size_t i = 0; i < 6 && i < p0.positions.size() && i < p1.positions.size(); ++i) {
          current_positions_[i] = (1.0 - alpha) * p0.positions[i] + alpha * p1.positions[i];
          if (i < p0.velocities.size() && i < p1.velocities.size()) {
            current_velocities_[i] = (1.0 - alpha) * p0.velocities[i] + alpha * p1.velocities[i];
          } else if (dt_seg > 1e-6) {
            current_velocities_[i] = (p1.positions[i] - p0.positions[i]) / dt_seg;
          }
        }
      }
    } else {
      std::fill(current_velocities_.begin(), current_velocities_.end(), 0.0);
    }

    // Smoothly interpolate gripper toward target (0.05 m/s maximum speed)
    double grip_diff = gripper_target_ - gripper_position_;
    double max_grip_step = 0.05 * 0.02;  // speed * dt
    if (std::abs(grip_diff) <= max_grip_step) {
      gripper_position_ = gripper_target_;
    } else {
      gripper_position_ += (grip_diff > 0.0 ? max_grip_step : -max_grip_step);
    }

    // 2. Publish Standard JointState Message (6 arm joints + 2 gripper joints)
    std::vector<double> full_positions = current_positions_;
    full_positions.push_back(gripper_position_);  // left_finger_joint
    full_positions.push_back(gripper_position_);  // right_finger_joint

    std::vector<double> full_velocities = current_velocities_;
    full_velocities.push_back(0.0);
    full_velocities.push_back(0.0);

    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = now;
    joint_msg.name = joint_names_;
    joint_msg.position = full_positions;
    joint_msg.velocity = full_velocities;
    joint_state_pub_->publish(joint_msg);

    // 3. Compute Real-Time Forward Kinematics for End-Effector Tracing (TCP at gripper tip)
    Eigen::VectorXd q(6);
    for (size_t i = 0; i < 6; ++i) {
      q[i] = current_positions_[i];
    }
    Eigen::Matrix4d T_flange = kinematics_.computeFK(q);
    Eigen::Matrix4d T_flange_to_tcp = Eigen::Matrix4d::Identity();
    T_flange_to_tcp(2, 3) = 0.112;  // TCP offset: +112 mm along tool Z from Link6 flange
    Eigen::Matrix4d T_ee = T_flange * T_flange_to_tcp;

    Eigen::Vector3d pos = T_ee.block<3, 1>(0, 3);
    Eigen::Matrix3d rot = T_ee.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rot);

    // 4. Publish End-Effector PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.position.z = pos.z();
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    ee_pose_pub_->publish(pose_msg);

    // 5. Append to Trace if Moved by at least 1 mm
    geometry_msgs::msg::Point pt;
    pt.x = pos.x();
    pt.y = pos.y();
    pt.z = pos.z();

    bool moved = false;
    if (trace_points_.empty()) {
      moved = true;
    } else {
      double dx = pt.x - trace_points_.back().x;
      double dy = pt.y - trace_points_.back().y;
      double dz = pt.z - trace_points_.back().z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.001) {  // 1 mm threshold
        moved = true;
      }
    }

    if (moved) {
      trace_points_.push_back(pt);
      trace_poses_.push_back(pose_msg);

      // Keep max 2000 points in trace buffer to prevent memory bloat
      if (trace_points_.size() > 2000) {
        trace_points_.erase(trace_points_.begin());
        trace_poses_.erase(trace_poses_.begin());
      }
    }

    // 6. Publish Visual 3D Marker (LINE_STRIP)
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now;
    marker.header.frame_id = "base_link";
    marker.ns = "end_effector_trace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.006;  // 6 mm thick vibrant ribbon
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.85;   // Bright Electric Cyan / Neon Green
    marker.color.a = 0.95;
    marker.points = trace_points_;
    trace_marker_pub_->publish(marker);

    // 7. Publish Nav Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = "base_link";
    path_msg.poses = trace_poses_;
    path_pub_->publish(path_msg);
  }

  // Member variables
  const std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> start_positions_;
  std::vector<double> current_velocities_;
  double gripper_position_{0.0};
  double gripper_target_{0.0};

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_queue_;
  rclcpp::Time traj_start_time_;
  bool has_active_goal_{false};
  std::mutex state_mutex_;

  dobot_kinematics::GeneralizedKinematicsEngine kinematics_;

  std::vector<geometry_msgs::msg::Point> trace_points_;
  std::vector<geometry_msgs::msg::PoseStamped> trace_poses_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trace_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dobot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dobot_gazebo::DobotSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
