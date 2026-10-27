#ifndef DOBOT_EXECUTIVE__EXPERIMENT_RUNNER_HPP_
#define DOBOT_EXECUTIVE__EXPERIMENT_RUNNER_HPP_

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dobot_executive/executive_engine.hpp"
#include "dobot_executive/experiment_task.hpp"

namespace dobot_executive
{

// ===========================================================================
// EXPERIMENT SUITE RUNNER (SRP & GoF Orchestration)
// ===========================================================================

class ExperimentSuiteRunner
{
public:
  explicit ExperimentSuiteRunner(
    std::shared_ptr<ConsoleTelemetryObserver> observer = nullptr,
    double stage_delay_ms = 500.0)
  : observer_(std::move(observer)),
    stage_delay_ms_(stage_delay_ms)
  {
  }

  void addTask(std::shared_ptr<IExperimentTask> task)
  {
    if (task) {
      tasks_.push_back(std::move(task));
    }
  }

  bool runAll(ExecutiveEngine & engine, int countdown_s = 5)
  {
    results_.clear();

    printHeader();

    if (countdown_s > 0) {
      runCountdown(countdown_s);
    }

    bool all_passed = true;

    for (size_t i = 0; i < tasks_.size(); ++i) {
      const auto & task = tasks_[i];

      std::cout << "\n=================================================================\n";
      std::cout << "  STAGE " << (i + 1) << "/" << tasks_.size() << ": " << task->getName() << "\n";
      std::cout << "  Objective: " << task->getObjective() << "\n";
      std::cout << "=================================================================\n";

      auto result = task->execute(engine, observer_);
      results_.push_back(result);

      printTaskReport(result);

      if (!result.success) {
        all_passed = false;
        std::cerr << "  [WARNING]: Task execution returned failure! Aborting suite for safety.\n";
        break;
      }

      if (stage_delay_ms_ > 0.0 && i + 1 < tasks_.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(stage_delay_ms_)));
      }
    }

    printSummary(all_passed);
    return all_passed;
  }

  [[nodiscard]] const std::vector<ExperimentTaskResult> & getResults() const noexcept
  {
    return results_;
  }

private:
  void printHeader() const
  {
    std::cout << "\n#################################################################\n";
    std::cout << "   DOBOT NOVA 5 ROBOTICS STACK: PHASE 5 ADVANCED EXPERIMENTS\n";
    std::cout << "   Orchestrated via Command Pattern & Observer Reactive Telemetry\n";
    std::cout << "#################################################################\n";
    for (size_t i = 0; i < tasks_.size(); ++i) {
      std::cout << "  [" << (i + 1) << "] " << tasks_[i]->getName() << "\n";
    }
    std::cout << "#################################################################\n";
  }

  void runCountdown(int seconds) const
  {
    std::cout << "\n=================================================================\n";
    std::cout << "  [SIMULATION WARM-UP]: Pausing " << seconds << ".0 seconds before motion...\n";
    std::cout << "  Allows RViz2 camera framing and ROS 2 topic discovery to complete.\n";
    std::cout << "=================================================================\n";
    for (int c = seconds; c > 0; --c) {
      std::cout << "  -> Launching experiment suite in " << c << " s...\n" << std::flush;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "  -> Commencing motion sequence!\n";
  }

  void printTaskReport(const ExperimentTaskResult & res) const
  {
    std::cout << "  [Task Diagnostics]:\n";
    std::cout << "    - Status:              " << (res.success ? "SUCCESS" : "FAILED") << "\n";
    std::cout << "    - Message:             " << res.status_message << "\n";
    if (res.ik_time_ms > 0.0) {
      std::cout << "    - IK Solve Time:       " << std::fixed << std::setprecision(2)
                << res.ik_time_ms << " ms\n";
    }
    if (res.path_waypoints_count > 0) {
      std::cout << "    - Discrete Waypoints:  " << res.path_waypoints_count << " path points\n";
      std::cout << "    - Trajectory Points:   " << res.trajectory_points_count
                << " samples (50 Hz)\n";
      std::cout << "    - Trajectory Duration: " << std::fixed << std::setprecision(3)
                << res.trajectory_duration_s << " s\n";
    }
    if (res.peak_vel_utilization_pct > 0.0) {
      std::cout << "    - Peak Joint Vel Util: " << std::fixed << std::setprecision(1)
                << res.peak_vel_utilization_pct << "% (Limit: 3.14 rad/s)\n";
      std::cout << "    - Peak Joint Acc Util: " << std::fixed << std::setprecision(1)
                << res.peak_acc_utilization_pct << "% (Limit: 15.0 rad/s^2)\n";
    }
    if (res.closure_or_return_error_mm > 0.0) {
      std::cout << "    - Precision Metric:    " << std::fixed << std::setprecision(4)
                << res.closure_or_return_error_mm << " mm error\n";
    }
  }

  void printSummary(bool all_passed) const
  {
    std::cout << "\n#################################################################\n";
    if (all_passed) {
      std::cout << "   ALL EXPERIMENTS COMPLETED SUCCESSFULLY WITH 100% PASS RATE!\n";
    } else {
      std::cout << "   EXPERIMENT SUITE FINISHED WITH SOME FAILURES\n";
    }
    std::cout << "=================================================================\n";
    std::cout << "  " << std::left << std::setw(5) << "#"
              << std::setw(42) << "Task Name"
              << std::setw(10) << "Status"
              << std::setw(12) << "Duration (s)"
              << "Error (mm)\n";
    std::cout << "-----------------------------------------------------------------\n";

    for (size_t i = 0; i < results_.size(); ++i) {
      const auto & r = results_[i];
      // Shorten name if too long
      std::string short_name = r.task_name;
      if (short_name.length() > 40) {
        short_name = short_name.substr(0, 37) + "...";
      }

      std::cout << "  " << std::left << std::setw(5) << (i + 1)
                << std::setw(42) << short_name
                << std::setw(10) << (r.success ? "SUCCESS" : "FAILED")
                << std::fixed << std::setprecision(2) << std::setw(12) << r.trajectory_duration_s
                << std::fixed << std::setprecision(4) << r.closure_or_return_error_mm << "\n";
    }
    std::cout << "#################################################################\n\n";
  }

  std::vector<std::shared_ptr<IExperimentTask>> tasks_;
  std::vector<ExperimentTaskResult> results_;
  std::shared_ptr<ConsoleTelemetryObserver> observer_;
  double stage_delay_ms_{500.0};
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__EXPERIMENT_RUNNER_HPP_

