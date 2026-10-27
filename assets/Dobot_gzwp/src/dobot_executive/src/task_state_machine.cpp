#include "dobot_executive/task_state_machine.hpp"

#include <iostream>
#include <thread>
#include "std_msgs/msg/empty.hpp"

namespace dobot_executive
{

TaskStateMachine::TaskStateMachine(std::shared_ptr<PickAndPlaceContext> context)
: context_(std::move(context))
{
  current_state_ = std::make_shared<IdleState>();
}

void TaskStateMachine::transitionTo(std::shared_ptr<ITaskState> next_state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!next_state) {
    return;
  }

  std::string from_name = current_state_ ? current_state_->getName() : "NONE";
  std::string to_name = next_state->getName();

  if (current_state_) {
    current_state_->onExit(*context_);
  }

  current_state_ = next_state;

  if (on_state_change_) {
    on_state_change_(from_name, to_name);
  }

  current_state_->onEnter(*context_);
}

StateResult TaskStateMachine::step()
{
  if (!current_state_) {
    return StateResult::FAILURE;
  }

  StateResult res = current_state_->execute(*context_);

  if (res == StateResult::EMERGENCY_STOP || res == StateResult::FAILURE) {
    if (current_state_->getName() != "12. SAFETY_RECOVERY") {
      transitionTo(std::make_shared<SafetyRecoveryState>());
    }
    return res;
  }

  if (res == StateResult::SUCCESS) {
    std::string s_name = current_state_->getName();

    if (s_name == "1. IDLE") {
      transitionTo(std::make_shared<PerceiveState>());
    } else if (s_name == "2. PERCEIVE") {
      context_->incrementCycle();
      transitionTo(std::make_shared<ApproachPrePickState>());
    } else if (s_name == "3. APPROACH_PRE_PICK") {
      transitionTo(std::make_shared<ApproachPickState>());
    } else if (s_name == "4. APPROACH_PICK") {
      transitionTo(std::make_shared<GraspState>());
    } else if (s_name == "5. GRASP") {
      transitionTo(std::make_shared<RetractPickState>());
    } else if (s_name == "6. RETRACT_PICK") {
      transitionTo(std::make_shared<TransportState>());
    } else if (s_name == "7. TRANSPORT") {
      transitionTo(std::make_shared<ApproachPlaceState>());
    } else if (s_name == "8. APPROACH_PLACE") {
      transitionTo(std::make_shared<ReleaseState>());
    } else if (s_name == "9. RELEASE") {
      transitionTo(std::make_shared<RetractPlaceState>());
    } else if (s_name == "10. RETRACT_PLACE") {
      if (context_->getCycleCount() < context_->getMaxCycles()) {
        std::cout << "\n=========================================================================\n";
        std::cout << "  [CYCLE COMPLETE] Pick-and-Place Cycle #" << context_->getCycleCount()
                  << " of " << context_->getMaxCycles() << " Finished.\n";
        std::cout << "  Respawning workpiece billet for Next Cycle in "
                  << context_->config().loop_restart_delay_s << "s...\n";
        std::cout << "=========================================================================\n";

        // Trigger simulator workpiece reset topic if available
        auto reset_pub = context_->node()->create_publisher<std_msgs::msg::Empty>("/dobot/reset_workpiece", 1);
        std_msgs::msg::Empty empty_msg;
        reset_pub->publish(empty_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(context_->config().loop_restart_delay_s * 1000.0)));

        // Advance to next cycle: perceive & approach pre-pick
        transitionTo(std::make_shared<PerceiveState>());
      } else {
        std::cout << "\n=========================================================================\n";
        std::cout << "  ALL " << context_->getMaxCycles() << " PICK-AND-PLACE CYCLES COMPLETED!\n";
        std::cout << "=========================================================================\n";
        transitionTo(std::make_shared<ReturnHomeState>());
      }
    } else if (s_name == "11. RETURN_HOME") {
      std::cout << "\n  [MISSION COMPLETED] Manipulator secured in canonical home posture.\n";
    }
  }

  return res;
}

bool TaskStateMachine::run(size_t num_cycles)
{
  std::cout << "\n=========================================================================\n";
  std::cout << "  STARTING AUTONOMOUS PICK-AND-PLACE MISSION (" << num_cycles << " CYCLES)\n";
  std::cout << "=========================================================================\n";

  // Initialize Idle State
  current_state_->onEnter(*context_);

  while (rclcpp::ok() && !context_->isAbortRequested()) {
    StateResult res = step();

    if (current_state_->getName() == "11. RETURN_HOME" && res == StateResult::SUCCESS) {
      return true;
    }

    if (current_state_->getName() == "12. SAFETY_RECOVERY" || res == StateResult::EMERGENCY_STOP) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return false;
}

void TaskStateMachine::abort()
{
  context_->requestAbort();
  transitionTo(std::make_shared<SafetyRecoveryState>());
}

std::string TaskStateMachine::getCurrentStateName() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_ ? current_state_->getName() : "UNKNOWN";
}

}  // namespace dobot_executive
