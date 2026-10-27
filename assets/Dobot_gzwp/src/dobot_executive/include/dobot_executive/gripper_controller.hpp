#ifndef DOBOT_EXECUTIVE__GRIPPER_CONTROLLER_HPP_
#define DOBOT_EXECUTIVE__GRIPPER_CONTROLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace dobot_executive
{

// ===========================================================================
// 1. INTERFACE SEGREGATION PRINCIPLE (ISP): IGripperController
// ===========================================================================

/**
 * @brief Abstract End-Effector Gripper Controller Interface (ISP).
 * Segregates gripper actuation, state inspection, and feedback from arm motion planning.
 */
class IGripperController
{
public:
  virtual ~IGripperController() = default;

  /**
   * @brief Open gripper jaws to maximum stroke aperture.
   * @param speed Linear jaw velocity (m/s)
   */
  virtual bool open(double speed = 0.05) = 0;

  /**
   * @brief Close gripper jaws to zero aperture (or until grasp force reached).
   * @param speed Linear jaw velocity (m/s)
   */
  virtual bool close(double speed = 0.05) = 0;

  /**
   * @brief Set precise jaw aperture width in meters.
   * @param width_m Target opening stroke (meters)
   * @param speed Linear jaw velocity (m/s)
   */
  virtual bool setWidth(double width_m, double speed = 0.05) = 0;

  /**
   * @brief Get instantaneous measured finger displacement in meters.
   */
  [[nodiscard]] virtual double getCurrentWidth() const = 0;

  /**
   * @brief Get maximum allowable finger displacement stroke in meters.
   */
  [[nodiscard]] virtual double getMaxWidth() const = 0;

  /**
   * @brief Check whether gripper is actively moving towards target width.
   */
  [[nodiscard]] virtual bool isMoving() const = 0;

  /**
   * @brief Check whether gripper has stalled on an object before reaching target.
   */
  [[nodiscard]] virtual bool isGrasping() const = 0;

  /**
   * @brief Block until jaw motion finishes or timeout occurs.
   */
  virtual bool waitForMotionComplete(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) = 0;
};

// ===========================================================================
// 2. CONCRETE IMPLEMENTATION: SimulatedGripperController
// ===========================================================================

/**
 * @brief Gripper controller for simulated robot in Gazebo / RViz2.
 * Publishes jaw commands on `/dobot/gripper/command` (std_msgs::msg::Float64)
 * and monitors real-time prismatic jaw feedback from `/joint_states`.
 */
class SimulatedGripperController : public IGripperController
{
public:
  explicit SimulatedGripperController(
    rclcpp::Node::SharedPtr node,
    std::string command_topic = "/dobot/gripper/command",
    std::string joint_states_topic = "/joint_states",
    double max_width = 0.025)
  : node_(std::move(node)),
    command_topic_(std::move(command_topic)),
    joint_states_topic_(std::move(joint_states_topic)),
    max_width_(max_width),
    target_width_(max_width)
  {
    cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(command_topic_, 10);

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      10,
      std::bind(&SimulatedGripperController::jointStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "SimulatedGripperController initialized. Command topic: %s, Max stroke: %.1f mm",
      command_topic_.c_str(), max_width_ * 1000.0);
  }

  bool open(double /*speed*/ = 0.05) override
  {
    return setWidth(max_width_);
  }

  bool close(double /*speed*/ = 0.05) override
  {
    return setWidth(0.0);
  }

  bool setWidth(double width_m, double /*speed*/ = 0.05) override
  {
    double clamped_width = std::clamp(width_m, 0.0, max_width_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_width_ = clamped_width;
    }

    std_msgs::msg::Float64 msg;
    msg.data = clamped_width;
    cmd_pub_->publish(msg);
    return true;
  }

  [[nodiscard]] double getCurrentWidth() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_width_;
  }

  [[nodiscard]] double getMaxWidth() const noexcept override
  {
    return max_width_;
  }

  [[nodiscard]] bool isMoving() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::abs(current_width_ - target_width_) > 0.0008;
  }

  [[nodiscard]] bool isGrasping() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // In simulation without contact sensors, grasping can be detected if closed target > 0
    // or when near closed threshold
    return (current_width_ > 0.002 && current_width_ < (max_width_ - 0.002));
  }

  bool waitForMotionComplete(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) override
  {
    const auto start_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(20);

    while (isMoving()) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        return false;
      }
      std::this_thread::sleep_for(interval);
    }
    return true;
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_finger_joint" && i < msg->position.size()) {
        current_width_ = msg->position[i];
        break;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string command_topic_;
  std::string joint_states_topic_;
  double max_width_{0.025};
  double target_width_{0.025};
  double current_width_{0.025};

  mutable std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__GRIPPER_CONTROLLER_HPP_

