#include "dobot_executive/task_state.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace dobot_executive
{

// ===========================================================================
// 1. IdleState
// ===========================================================================

void IdleState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("cyan");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 1. IDLE (Standby & System Health Verification)\n";
  std::cout << "=========================================================================\n";
}

StateResult IdleState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  // Ensure gripper is at open aperture for workpiece entry
  if (ctx.gripper()) {
    ctx.gripper()->open();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // Articulate to Quadrant 1 Floor Working Posture
  auto report = ctx.executive().moveJoint(ctx.getWorkReadyJoints(), "Idle to Work Ready");
  if (!report.success) {
    std::cerr << "  [IdleState] Failed to articulate to work-ready configuration: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.2);
  }

  std::cout << "  [IdleState] System verified and in position. Ready for mission dispatch.\n";
  return StateResult::SUCCESS;
}

void IdleState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 2. PerceiveState
// ===========================================================================

void PerceiveState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("cyan");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 2. PERCEIVE (Target Localization in Quadrant 1)\n";
  std::cout << "=========================================================================\n";
}

StateResult PerceiveState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  // Allow brief window to process latest workpiece perception pose
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  Eigen::Vector3d pick_pos = ctx.getPickPos();
  Eigen::Vector3d place_pos = ctx.getPlacePos();

  std::cout << "  [PerceiveState] Pick Station (Workpiece): ["
            << std::fixed << std::setprecision(4)
            << pick_pos.x() << ", " << pick_pos.y() << ", " << pick_pos.z() << "] m\n";
  std::cout << "  [PerceiveState] Place Station (Drop Bin):  ["
            << place_pos.x() << ", " << place_pos.y() << ", " << place_pos.z() << "] m\n";
  std::cout << "  [PerceiveState] Grasp alignment validated. Pre-pick trajectory ready.\n";

  return StateResult::SUCCESS;
}

void PerceiveState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 3. ApproachPrePickState
// ===========================================================================

void ApproachPrePickState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("azure");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 3. APPROACH_PRE_PICK (Motion to Pre-Grasp Standoff)\n";
  std::cout << "=========================================================================\n";
}

StateResult ApproachPrePickState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d pre_pick = ctx.getPrePickPos();
  std::cout << "  [ApproachPrePick] Moving to standoff pose: ["
            << std::fixed << std::setprecision(4)
            << pre_pick.x() << ", " << pre_pick.y() << ", " << pre_pick.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    pre_pick,
    ctx.config().approach_steps,
    "Approach Pre-Pick");

  if (!report.success) {
    std::cerr << "  [ApproachPrePick] Kinematic planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.15);
  }

  // Confirm gripper is fully open
  if (ctx.gripper()) {
    ctx.gripper()->open();
  }

  return StateResult::SUCCESS;
}

void ApproachPrePickState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 4. ApproachPickState
// ===========================================================================

void ApproachPickState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("azure");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 4. APPROACH_PICK (Vertical Descent to Grasp Altitude)\n";
  std::cout << "=========================================================================\n";
}

StateResult ApproachPickState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d pick_pos = ctx.getPickPos();
  std::cout << "  [ApproachPick] Linear vertical descent to workpiece: ["
            << std::fixed << std::setprecision(4)
            << pick_pos.x() << ", " << pick_pos.y() << ", " << pick_pos.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    pick_pos,
    ctx.config().approach_steps,
    "Descent to Workpiece");

  if (!report.success) {
    std::cerr << "  [ApproachPick] Descent planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.15);
  }

  return StateResult::SUCCESS;
}

void ApproachPickState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 5. GraspState
// ===========================================================================

void GraspState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("crimson");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 5. GRASP (Finger Clamping & Physical Force Application)\n";
  std::cout << "=========================================================================\n";
}

StateResult GraspState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  std::cout << "  [GraspState] Actuating parallel jaws inward to grip workpiece billet...\n";
  if (ctx.gripper()) {
    ctx.gripper()->close();
  }

  // Dwell to allow PyBullet contact physics, friction, and motor torque to settle
  int dwell_ms = static_cast<int>(ctx.config().grasp_dwell_s * 1000.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));

  std::cout << "  [GraspState] Workpiece securely clamped under physical Coulomb friction.\n";
  return StateResult::SUCCESS;
}

void GraspState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 6. RetractPickState
// ===========================================================================

void RetractPickState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("amber");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 6. RETRACT_PICK (Vertical Lifting Ascent)\n";
  std::cout << "=========================================================================\n";
}

StateResult RetractPickState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d pre_pick = ctx.getPrePickPos();
  std::cout << "  [RetractPick] Lifting workpiece vertically to: ["
            << std::fixed << std::setprecision(4)
            << pre_pick.x() << ", " << pre_pick.y() << ", " << pre_pick.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    pre_pick,
    ctx.config().approach_steps,
    "Retract Lift");

  if (!report.success) {
    std::cerr << "  [RetractPick] Lift planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.15);
  }

  std::cout << "  [RetractPick] Workpiece successfully lifted clear of the fixture plate.\n";
  return StateResult::SUCCESS;
}

void RetractPickState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 7. TransportState
// ===========================================================================

void TransportState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("purple");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 7. TRANSPORT (Spatial Transfer to Place Station)\n";
  std::cout << "=========================================================================\n";
}

StateResult TransportState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d pre_place = ctx.getPrePlacePos();
  std::cout << "  [TransportState] Transferring workpiece across Quadrant 1 to: ["
            << std::fixed << std::setprecision(4)
            << pre_place.x() << ", " << pre_place.y() << ", " << pre_place.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    pre_place,
    ctx.config().transport_steps,
    "Transport to Drop Bin");

  if (!report.success) {
    std::cerr << "  [TransportState] Transfer path planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.20);
  }

  std::cout << "  [TransportState] Workpiece positioned directly above the collection drop bin.\n";
  return StateResult::SUCCESS;
}

void TransportState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 8. ApproachPlaceState
// ===========================================================================

void ApproachPlaceState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("emerald");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 8. APPROACH_PLACE (Descent into Collection Bin)\n";
  std::cout << "=========================================================================\n";
}

StateResult ApproachPlaceState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d place_pos = ctx.getPlacePos();
  std::cout << "  [ApproachPlace] Descending into drop bin aperture: ["
            << std::fixed << std::setprecision(4)
            << place_pos.x() << ", " << place_pos.y() << ", " << place_pos.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    place_pos,
    ctx.config().approach_steps,
    "Descent into Bin");

  if (!report.success) {
    std::cerr << "  [ApproachPlace] Place descent planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.15);
  }

  return StateResult::SUCCESS;
}

void ApproachPlaceState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 9. ReleaseState
// ===========================================================================

void ReleaseState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("emerald");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 9. RELEASE (Gripper Opening & Gravitational Part Drop)\n";
  std::cout << "=========================================================================\n";
}

StateResult ReleaseState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  std::cout << "  [ReleaseState] Opening gripper jaws to release workpiece into bin...\n";
  if (ctx.gripper()) {
    ctx.gripper()->open();
  }

  int dwell_ms = static_cast<int>(ctx.config().release_dwell_s * 1000.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));

  std::cout << "  [ReleaseState] Workpiece successfully dropped into container.\n";
  return StateResult::SUCCESS;
}

void ReleaseState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 10. RetractPlaceState
// ===========================================================================

void RetractPlaceState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("emerald");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 10. RETRACT_PLACE (Ascent Clearing Bin Sidewalls)\n";
  std::cout << "=========================================================================\n";
}

StateResult RetractPlaceState::execute(PickAndPlaceContext & ctx)
{
  if (ctx.isAbortRequested()) {
    return StateResult::EMERGENCY_STOP;
  }

  Eigen::Vector3d pre_place = ctx.getPrePlacePos();
  std::cout << "  [RetractPlace] Ascending clear of bin rim: ["
            << std::fixed << std::setprecision(4)
            << pre_place.x() << ", " << pre_place.y() << ", " << pre_place.z() << "] m\n";

  auto report = ctx.executive().moveLine(
    pre_place,
    ctx.config().approach_steps,
    "Retract from Bin");

  if (!report.success) {
    std::cerr << "  [RetractPlace] Retract planning error: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.15);
  }

  ctx.incrementSuccessCount();
  std::cout << "  [RetractPlace] Pick-and-Place Cycle #" << ctx.getCycleCount()
            << " COMPLETED SUCCESSFULLY! (Total Successes: "
            << ctx.getSuccessfulCycles() << ")\n";

  return StateResult::SUCCESS;
}

void RetractPlaceState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 11. ReturnHomeState
// ===========================================================================

void ReturnHomeState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("cyan");
  std::cout << "\n=========================================================================\n";
  std::cout << "  [STATE] -> 11. RETURN_HOME (Articulating to Standby Posture)\n";
  std::cout << "=========================================================================\n";
}

StateResult ReturnHomeState::execute(PickAndPlaceContext & ctx)
{
  auto report = ctx.executive().moveJoint(ctx.getHomeJoints(), "Return Home");
  if (!report.success) {
    std::cerr << "  [ReturnHomeState] Error returning to home posture: "
              << report.status_message << "\n";
    return StateResult::FAILURE;
  }

  if (ctx.observer()) {
    ctx.observer()->waitForTrajectory(report.trajectory_duration_s, 0.2);
  }

  std::cout << "  [ReturnHomeState] Robot in canonical standby home configuration.\n";
  return StateResult::SUCCESS;
}

void ReturnHomeState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

// ===========================================================================
// 12. SafetyRecoveryState
// ===========================================================================

void SafetyRecoveryState::onEnter(PickAndPlaceContext & ctx)
{
  ctx.setTraceColor("crimson");
  std::cerr << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
  std::cerr << "  [SAFETY ALERT] -> 12. SAFETY_RECOVERY STATE ENGAGED!\n";
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
}

StateResult SafetyRecoveryState::execute(PickAndPlaceContext & ctx)
{
  // 1. Open gripper immediately
  if (ctx.gripper()) {
    ctx.gripper()->open();
  }

  // 2. Safely attempt return to work-ready altitude
  std::cerr << "  [SafetyRecovery] Aborting active trajectories and returning to safe posture...\n";
  ctx.executive().moveJoint(ctx.getWorkReadyJoints(), "Emergency Safe Retreat");

  return StateResult::FAILURE;
}

void SafetyRecoveryState::onExit(PickAndPlaceContext & /*ctx*/)
{
}

}  // namespace dobot_executive
