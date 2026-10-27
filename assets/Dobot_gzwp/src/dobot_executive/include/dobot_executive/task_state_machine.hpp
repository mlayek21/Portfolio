#ifndef DOBOT_EXECUTIVE__TASK_STATE_MACHINE_HPP_
#define DOBOT_EXECUTIVE__TASK_STATE_MACHINE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dobot_executive/pick_and_place_context.hpp"
#include "dobot_executive/task_state.hpp"

namespace dobot_executive
{

/**
 * @brief Industrial Task State Machine Manager (Context & Orchestrator in State Pattern).
 * Manages deterministic state transitions, execution loops, safety guards, and telemetry.
 */
class TaskStateMachine
{
public:
  using StateChangeCallback = std::function<void(const std::string & from_state, const std::string & to_state)>;

  explicit TaskStateMachine(std::shared_ptr<PickAndPlaceContext> context);

  /**
   * @brief Perform transition to next discrete task state.
   */
  void transitionTo(std::shared_ptr<ITaskState> next_state);

  /**
   * @brief Execute a single tick of the active state and manage state transitions.
   */
  StateResult step();

  /**
   * @brief Run complete multi-cycle pick-and-place mission.
   * @param num_cycles Total number of pick-and-place repetitions
   */
  bool run(size_t num_cycles = 1);

  /**
   * @brief Request immediate safety halt and transition to SafetyRecoveryState.
   */
  void abort();

  /**
   * @brief Register callback hook for state transition events.
   */
  void setOnStateChange(StateChangeCallback cb)
  {
    on_state_change_ = std::move(cb);
  }

  [[nodiscard]] std::string getCurrentStateName() const;
  [[nodiscard]] std::shared_ptr<PickAndPlaceContext> getContext() const noexcept
  {
    return context_;
  }

private:
  std::shared_ptr<PickAndPlaceContext> context_;
  std::shared_ptr<ITaskState> current_state_;
  mutable std::mutex state_mutex_;
  StateChangeCallback on_state_change_;
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__TASK_STATE_MACHINE_HPP_
