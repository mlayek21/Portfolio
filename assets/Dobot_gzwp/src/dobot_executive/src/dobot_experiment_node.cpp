#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dobot_executive/executive_engine.hpp"
#include "dobot_executive/gripper_controller.hpp"
#include "dobot_executive/telemetry_observer.hpp"
#include "dobot_executive/experiment_task.hpp"
#include "dobot_executive/experiment_runner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dobot_experiment_node");

  // =========================================================================
  // 1. DEPENDENCY INJECTION & INTERFACE SEGREGATION
  // =========================================================================
  // Concrete Gripper Controller (ISP)
  auto gripper = std::make_shared<dobot_executive::SimulatedGripperController>(node);

  // Reactive Telemetry Observer (GoF Observer Pattern)
  auto observer = std::make_shared<dobot_executive::ConsoleTelemetryObserver>(0.15);

  dobot_executive::ExecutiveConfig config;
  config.auto_execute = true;
  config.sample_dt = 0.02;  // 50 Hz control loop

  // Master Executive Engine Facade (DIP)
  dobot_executive::ExecutiveEngine executive(node, config);
  executive.setGripperController(gripper);
  executive.addObserver(observer);

  // =========================================================================
  // 2. CONCURRENT ROS 2 EVENT LOOP
  // =========================================================================
  std::thread spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Allow ROS 2 topic discovery to establish
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  // =========================================================================
  // 3. COMMAND PATTERN SUITE COMPOSITION & EXECUTION
  // =========================================================================
  dobot_executive::ExperimentSuiteRunner runner(observer);

  // Stage 0: Initial Posture Setup (+X, +Y Quadrant 1, tool strictly perpendicular)
  runner.addTask(std::make_shared<dobot_executive::SetupWorkingPoseTask>());

  // Stage 1: Experiment 5.1 (MoveL in XY Plane: +180mm along X & Return)
  runner.addTask(std::make_shared<dobot_executive::LineExperimentTask>(0.180, 40));

  // Stage 2: Experiment 5.2 (MoveC in XY Plane: 160mm dia circle & Return)
  runner.addTask(std::make_shared<dobot_executive::CircleExperimentTask>(0.080, 50));

  // Stage 3: Experiment 5.3 (MoveRect in XY Plane: 160mm x 100mm rectangle & Return)
  runner.addTask(std::make_shared<dobot_executive::RectangleExperimentTask>(0.160, 0.100, 15));

  // Stage 4: Experiment 5.4 (MoveSpiral in XY Plane: 2D Archimedean spiral & Return)
  runner.addTask(std::make_shared<dobot_executive::SpiralExperimentTask>(0.110, 2.0, 60));

  // Stage 5: Experiment 5.5 (MoveHelix in 3D: Cylindrical helix +60mm Z rise & Return)
  runner.addTask(std::make_shared<dobot_executive::HelixExperimentTask>(0.060, 2.0, 60));

  // Stage 6: Gripper Dual-Finger Actuation Validation (Open -> Close -> Nominal)
  runner.addTask(std::make_shared<dobot_executive::GripperValidationTask>());

  // Stage 7: Canonical Home Pose Articulation
  runner.addTask(std::make_shared<dobot_executive::ReturnHomeTask>());

  // Execute complete orchestrated experiment suite with 5s warm-up countdown
  runner.runAll(executive, 5);

  // =========================================================================
  // 4. CLEAN SHUTDOWN
  // =========================================================================
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
