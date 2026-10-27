#ifndef DOBOT_EXECUTIVE__TELEMETRY_OBSERVER_HPP_
#define DOBOT_EXECUTIVE__TELEMETRY_OBSERVER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dobot_executive
{

// Forward declaration
struct TaskExecutionReport;

// ===========================================================================
// 1. OBSERVER PATTERN: ITelemetryObserver
// ===========================================================================

/**
 * @brief Abstract Telemetry Observer (GoF Observer Pattern).
 * Receives reactive event callbacks from ExecutiveEngine for state changes,
 * trajectory dispatches, and execution lifecycle events.
 */
class ITelemetryObserver
{
public:
  virtual ~ITelemetryObserver() = default;

  /**
   * @brief Invoked reactively when new joint states arrive and FK pose is updated.
   * @param joints Instantaneous N-DOF joint positions
   * @param tcp_pose Forward kinematics homogeneous transform T_0^tcp
   */
  virtual void onJointStateUpdated(
    const Eigen::VectorXd & joints,
    const Eigen::Matrix4d & tcp_pose) = 0;

  /**
   * @brief Invoked when a trajectory is sent to the robot controller.
   * @param task_name Identifier of the active task
   * @param duration_s Planned trajectory duration in seconds
   * @param trajectory_points_count Number of 50 Hz trajectory samples
   */
  virtual void onTrajectoryDispatched(
    const std::string & task_name,
    double duration_s,
    size_t trajectory_points_count) = 0;

  /**
   * @brief Invoked when an executed trajectory finishes or returns status.
   * @param task_name Identifier of the completed task
   * @param report Detailed diagnostics and limit utilization report
   */
  virtual void onExecutionComplete(
    const std::string & task_name,
    const TaskExecutionReport & report) = 0;
};

// ===========================================================================
// 2. CONCRETE OBSERVER: ConsoleTelemetryObserver
// ===========================================================================

/**
 * @brief Real-time console streaming observer.
 * Reactively formats and streams Cartesian Tool Center Point coordinates
 * and execution telemetry without blocking polling loops.
 */
class ConsoleTelemetryObserver : public ITelemetryObserver
{
public:
  explicit ConsoleTelemetryObserver(double stream_interval_s = 0.15)
  : stream_interval_s_(stream_interval_s)
  {
  }

  void onTrajectoryDispatched(
    const std::string & task_name,
    double duration_s,
    size_t trajectory_points_count) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_task_name_ = task_name;
    expected_duration_s_ = duration_s;
    trajectory_points_count_ = trajectory_points_count;
    start_time_ = std::chrono::steady_clock::now();
    last_stream_time_ = start_time_;
    is_tracking_ = true;

    std::cout << "    -> Live Controller Tracking & Cartesian Telemetry [" << task_name << "]:\n";
  }

  void onJointStateUpdated(
    const Eigen::VectorXd & /*joints*/,
    const Eigen::Matrix4d & tcp_pose) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_tracking_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - start_time_).count();
    double since_last_s = std::chrono::duration<double>(now - last_stream_time_).count();

    if (since_last_s >= stream_interval_s_ || elapsed_s >= expected_duration_s_) {
      last_stream_time_ = now;
      Eigen::Vector3d p_tcp = tcp_pose.block<3, 1>(0, 3);

      std::cout << "       [t=" << std::fixed << std::setprecision(2) << std::setw(4) << elapsed_s
                << "s] EE Position: X=" << std::setprecision(4) << std::setw(8) << p_tcp.x()
                << " m | Y=" << std::setprecision(4) << std::setw(8) << p_tcp.y()
                << " m | Z=" << std::setprecision(4) << std::setw(8) << p_tcp.z() << " m\n"
                << std::flush;
    }
  }

  void onExecutionComplete(
    const std::string & task_name,
    const TaskExecutionReport & report) override;

  /**
   * @brief Wait for active trajectory duration to elapse with extra safety buffer.
   */
  void waitForTrajectory(double duration_s, double buffer_s = 0.3)
  {
    const int total_ms = static_cast<int>((duration_s + buffer_s) * 1000.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(total_ms));
    std::lock_guard<std::mutex> lock(mutex_);
    is_tracking_ = false;
  }

  void stopTracking()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_tracking_ = false;
  }

private:
  double stream_interval_s_{0.15};
  mutable std::mutex mutex_;
  bool is_tracking_{false};
  std::string active_task_name_;
  double expected_duration_s_{0.0};
  size_t trajectory_points_count_{0};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_stream_time_;
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__TELEMETRY_OBSERVER_HPP_

