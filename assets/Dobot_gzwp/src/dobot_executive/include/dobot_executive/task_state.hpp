#ifndef DOBOT_EXECUTIVE__TASK_STATE_HPP_
#define DOBOT_EXECUTIVE__TASK_STATE_HPP_

#include <memory>
#include <string>

#include "dobot_executive/pick_and_place_context.hpp"

namespace dobot_executive
{

/**
 * @brief Discrete outcome of a state execution tick.
 */
enum class StateResult
{
  SUCCESS,         ///< State completed all objectives successfully
  IN_PROGRESS,     ///< State is currently executing an asynchronous sub-task
  RETRY,           ///< Transient anomaly occurred; state requests re-execution
  FAILURE,         ///< Kinematic or actuator failure; trigger recovery
  EMERGENCY_STOP   ///< Critical safety violation; abort entire cycle immediately
};

/**
 * @brief Abstract State Pattern Interface (GoF State Pattern).
 * Encapsulates the behavior, preconditions, motion generation, and exit hooks for an industrial task state.
 */
class ITaskState
{
public:
  virtual ~ITaskState() = default;

  [[nodiscard]] virtual std::string getName() const = 0;
  [[nodiscard]] virtual std::string getDescription() const = 0;

  virtual void onEnter(PickAndPlaceContext & ctx) = 0;
  virtual StateResult execute(PickAndPlaceContext & ctx) = 0;
  virtual void onExit(PickAndPlaceContext & ctx) = 0;
};

// ===========================================================================
// CONCRETE TASK STATES DECLARATIONS
// ===========================================================================

/**
 * @brief State 1: Standby and Mission Readiness Check.
 */
class IdleState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "1. IDLE"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Standby in ready posture; verify hardware controllers and telemetry.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 2: Perceive and resolve target workpiece coordinates in Quadrant 1.
 */
class PerceiveState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "2. PERCEIVE"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Acquire workpiece location from telemetry and calibrate grasp alignment.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 3: Move to vertical standoff position above workpiece with open jaws.
 */
class ApproachPrePickState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "3. APPROACH_PRE_PICK"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Articulate arm to +120mm standoff above pick target and open jaws.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 4: Precision vertical descent along -Z into workpiece grasp altitude.
 */
class ApproachPickState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "4. APPROACH_PICK"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Linear vertical descent along -Z until gripper jaws envelop the workpiece.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 5: Close gripper fingers and hold force dwell to secure workpiece.
 */
class GraspState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "5. GRASP"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Command parallel gripper fingers closed and apply clamping force.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 6: Vertical lift along +Z raising grasped workpiece off the table.
 */
class RetractPickState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "6. RETRACT_PICK"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Linear vertical ascent along +Z to lift workpiece clear of the fixture plate.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 7: Spatial Cartesian transfer trajectory to drop bin standoff in Quadrant 1.
 */
class TransportState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "7. TRANSPORT"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Execute smooth C2 continuous transfer trajectory to pre-place bin standoff.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 8: Vertical descent into the open aperture of the collection drop bin.
 */
class ApproachPlaceState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "8. APPROACH_PLACE"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Linear vertical descent into the receptacle aperture of the drop bin.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 9: Command parallel fingers open to release workpiece into the bin.
 */
class ReleaseState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "9. RELEASE"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Open gripper jaws to 25mm to release workpiece under gravity into the bin.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 10: Vertical ascent clearing drop bin perimeter walls.
 */
class RetractPlaceState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "10. RETRACT_PLACE"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Linear vertical ascent along +Z clearing the sidewalls of the drop bin.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 11: Return arm to canonical standby home configuration.
 */
class ReturnHomeState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "11. RETURN_HOME"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Articulate manipulator safely back to canonical standby posture.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

/**
 * @brief State 12: Emergency Stop & Fault Recovery Handler.
 */
class SafetyRecoveryState : public ITaskState
{
public:
  [[nodiscard]] std::string getName() const override { return "12. SAFETY_RECOVERY"; }
  [[nodiscard]] std::string getDescription() const override
  {
    return "Emergency halt, release gripper, and articulate to safe retreat altitude.";
  }
  void onEnter(PickAndPlaceContext & ctx) override;
  StateResult execute(PickAndPlaceContext & ctx) override;
  void onExit(PickAndPlaceContext & ctx) override;
};

}  // namespace dobot_executive

#endif  // DOBOT_EXECUTIVE__TASK_STATE_HPP_
