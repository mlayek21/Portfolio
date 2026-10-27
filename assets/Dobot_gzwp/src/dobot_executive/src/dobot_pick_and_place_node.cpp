#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "dobot_executive/executive_engine.hpp"
#include "dobot_executive/gripper_controller.hpp"
#include "dobot_executive/telemetry_observer.hpp"
#include "dobot_executive/pick_and_place_context.hpp"
#include "dobot_executive/task_state_machine.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dobot_pick_and_place_node");

  // Parameters
  node->declare_parameter("num_cycles", 3);
  node->declare_parameter("countdown_seconds", 3);
  node->declare_parameter("auto_start", true);

  int num_cycles = node->get_parameter("num_cycles").as_int();
  int countdown_sec = node->get_parameter("countdown_seconds").as_int();
  bool auto_start = node->get_parameter("auto_start").as_bool();

  std::cout << "=========================================================================\n";
  std::cout << "  DOBOT NOVA 5 AUTONOMOUS PICK-AND-PLACE STATE MACHINE (PHASE 6)\n";
  std::cout << "  - Configured Cycles: " << num_cycles << "\n";
  std::cout << "  - Architecture:     GoF State Pattern (ITaskState & TaskStateMachine)\n";
  std::cout << "  - Workspace:        Quadrant 1 (+X, +Y Floor Inspection Plate)\n";
  std::cout << "  - Gripper:          Dual Prismatic Parallel Jaws (0.0 to 0.025m)\n";
  std::cout << "=========================================================================\n";

  // 1. Dependency Injection: Gripper, Telemetry Observer, and Executive Engine (DIP)
  auto gripper = std::make_shared<dobot_executive::SimulatedGripperController>(node);
  auto observer = std::make_shared<dobot_executive::ConsoleTelemetryObserver>(0.15);

  dobot_executive::ExecutiveConfig config;
  config.auto_execute = true;
  config.sample_dt = 0.02;  // 50 Hz control loop

  dobot_executive::ExecutiveEngine executive(node, config);
  executive.setGripperController(gripper);
  executive.addObserver(observer);

  // 2. Setup Context & State Machine
  dobot_executive::PickAndPlaceConfig pnp_cfg;
  pnp_cfg.max_cycles = static_cast<size_t>(num_cycles);

  auto context = std::make_shared<dobot_executive::PickAndPlaceContext>(
    node, executive, gripper, observer, pnp_cfg);

  // Status Publisher & Trace Color Publisher
  auto status_pub = node->create_publisher<std_msgs::msg::String>("/dobot/state_machine/status", 10);
  auto trace_pub = node->create_publisher<std_msgs::msg::String>("/dobot/trace_color", 10);
  context->setTraceColorPublisher(trace_pub);

  auto sm = std::make_shared<dobot_executive::TaskStateMachine>(context);

  sm->setOnStateChange([&](const std::string & from, const std::string & to) {
    std_msgs::msg::String status_msg;
    status_msg.data = to;
    status_pub->publish(status_msg);
    (void)from;
  });

  // 3. Workpiece Perception Subscription
  auto wp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/dobot/workpiece_pose", 10,
    [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      context->updateWorkpiecePose(p);
    });

  // 4. Concurrent ROS 2 Event Loop
  std::thread spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Discovery settling delay
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  // 5. Countdown Warm-Up
  if (countdown_sec > 0) {
    std::cout << "\n>>> Starting mission countdown in " << countdown_sec << " seconds...\n";
    for (int t = countdown_sec; t > 0; --t) {
      std::cout << "    [" << t << " s] Preparing workcell and kinematics...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << ">>> GO! Executing autonomous pick-and-place...\n\n";
  }

  // 6. Execute Mission
  if (auto_start) {
    bool ok = sm->run(static_cast<size_t>(num_cycles));
    std::cout << "\n=========================================================================\n";
    std::cout << "  MISSION " << (ok ? "COMPLETED SUCCESSFULLY!" : "TERMINATED WITH ANOMALIES.") << "\n";
    std::cout << "  Total Completed Cycles: " << context->getSuccessfulCycles()
              << " / " << num_cycles << "\n";
    std::cout << "=========================================================================\n";
  }

  // Clean Shutdown
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
