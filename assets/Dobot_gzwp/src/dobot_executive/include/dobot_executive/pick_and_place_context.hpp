#ifndef DOBOT_EXECUTIVE__PICK_AND_PLACE_CONTEXT_HPP_
#define DOBOT_EXECUTIVE__PICK_AND_PLACE_CONTEXT_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dobot_executive/executive_engine.hpp"
#include "dobot_executive/gripper_controller.hpp"
#include "dobot_executive/telemetry_observer.hpp"

namespace dobot_executive
{

/**
 * @brief Configuration parameters for Autonomous Pick-and-Place Cycle.
 */
struct PickAndPlaceConfig
{
  double standoff_z{0.120};          ///< Vertical standoff elevation above target (m)
  double pick_tcp_z{0.038};          ///< TCP altitude at grasp contact (m)
  double place_tcp_z{0.075};         ///< TCP altitude at bin drop release (m)
  double grasp_dwell_s{0.50};        ///< Settling dwell after finger closure (s)
  double release_dwell_s{0.40};      ///< Settling dwell after finger opening (s)
  size_t approach_steps{20};         ///< Discrete waypoints for vertical approach
  size_t transport_steps{35};        ///< Discrete waypoints for spatial transfer
  size_t max_cycles{3};              ///< Number of pick-and-place cycles to execute
  double loop_restart_delay_s{1.0};  ///< Dwell between consecutive cycles (s)
};

/**
 * @brief Shared Execution Context for GoF State Machine (Context in State Pattern).
 * Holds operational state, kinematic targets, engine references, and telemetry counters.
 */
class PickAndPlaceContext
{
public:
  PickAndPlaceContext(
    rclcpp::Node::SharedPtr node,
    ExecutiveEngine & engine,
    std::shared_ptr<IGripperController> gripper_ctrl,
    std::shared_ptr<ConsoleTelemetryObserver> obs = nullptr,
    PickAndPlaceConfig cfg = PickAndPlaceConfig())
  : node_(std::move(node)),
    executive_(engine),
    gripper_(std::move(gripper_ctrl)),
    observer_(std::move(obs)),
    config_(cfg)
  {
    // Canonical tool orientation: Tool Z strictly pointing down [0, 0, -1]
    R_perpendicular_ << 1.0,  0.0,  0.0,
                        0.0, -1.0,  0.0,
                        0.0,  0.0, -1.0;

    // Nominal station coordinates in Quadrant 1 (+X, +Y)
    pick_pos_ = Eigen::Vector3d(0.360, 0.080, config_.pick_tcp_z);
    place_pos_ = Eigen::Vector3d(0.380, 0.260, config_.place_tcp_z);

    // Canonical Standby Home Configuration
    home_joints_.resize(6);
    home_joints_ << -0.75, -0.30, 0.50, -0.20, 0.40, 0.0;

    // Quadrant 1 Floor Working Posture
    work_ready_joints_.resize(6);
    work_ready_joints_ << 0.8779, -0.6005, -1.8861, -2.2258, -1.5708, 2.4487;
  }

  // Engine & Controller References
  ExecutiveEngine & executive() noexcept { return executive_; }
  std::shared_ptr<IGripperController> gripper() noexcept { return gripper_; }
  std::shared_ptr<ConsoleTelemetryObserver> observer() noexcept { return observer_; }
  rclcpp::Node::SharedPtr node() noexcept { return node_; }
  const PickAndPlaceConfig & config() const noexcept { return config_; }

  // Target Poses
  [[nodiscard]] const Eigen::Matrix3d & getToolOrientation() const noexcept { return R_perpendicular_; }
  [[nodiscard]] const Eigen::Vector3d & getPickPos() const noexcept { return pick_pos_; }
  [[nodiscard]] const Eigen::Vector3d & getPlacePos() const noexcept { return place_pos_; }

  [[nodiscard]] Eigen::Vector3d getPrePickPos() const noexcept
  {
    return pick_pos_ + Eigen::Vector3d(0.0, 0.0, config_.standoff_z);
  }

  [[nodiscard]] Eigen::Vector3d getPrePlacePos() const noexcept
  {
    return place_pos_ + Eigen::Vector3d(0.0, 0.0, config_.standoff_z);
  }

  [[nodiscard]] const Eigen::VectorXd & getHomeJoints() const noexcept { return home_joints_; }
  [[nodiscard]] const Eigen::VectorXd & getWorkReadyJoints() const noexcept { return work_ready_joints_; }

  // Dynamic Perception Interface
  void updateWorkpiecePose(const Eigen::Vector3d & detected_pos)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pick_pos_.x() = detected_pos.x();
    pick_pos_.y() = detected_pos.y();
    pick_pos_.z() = config_.pick_tcp_z;
    has_perceived_workpiece_ = true;
  }

  [[nodiscard]] bool hasPerceivedWorkpiece() const noexcept
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return has_perceived_workpiece_;
  }

  // Mission Tracking & Metrics
  [[nodiscard]] size_t getCycleCount() const noexcept { return current_cycle_; }
  [[nodiscard]] size_t getMaxCycles() const noexcept { return config_.max_cycles; }
  void incrementCycle() noexcept { ++current_cycle_; }
  void incrementSuccessCount() noexcept { ++successful_cycles_; }
  [[nodiscard]] size_t getSuccessfulCycles() const noexcept { return successful_cycles_; }

  void requestAbort() noexcept { abort_requested_ = true; }
  [[nodiscard]] bool isAbortRequested() const noexcept { return abort_requested_; }

  void setTraceColor(const std::string & color_name)
  {
    if (trace_color_pub_) {
      std_msgs::msg::String msg;
      msg.data = color_name;
      trace_color_pub_->publish(msg);
    }
  }

  void setTraceColorPublisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub)
  {
    trace_color_pub_ = pub;
  }

private:
  rclcpp::Node::SharedPtr node_;
  ExecutiveEngine & executive_;
  std::shared_ptr<IGripperController> gripper_;
  std::shared_ptr<ConsoleTelemetryObserver> observer_;
  PickAndPlaceConfig config_;

  mutable std::mutex data_mutex_;
  Eigen::Matrix3d R_perpendicular_;
  Eigen::Vector3d pick_pos_;
  Eigen::Vector3d place_pos_;
  Eigen::VectorXd home_joints_;
  Eigen::VectorXd work_ready_joints_;

  bool has_perceived_workpiece_{false};
  size_t current_cycle_{0};
  size_t successful_cycles_{0};
  bool abort_requested_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trace_color_pub_;
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__PICK_AND_PLACE_CONTEXT_HPP_
