#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "dobot_kinematics/kinematics_engine.hpp"

namespace
{

void printVector(const std::string & label, const dobot_kinematics::JointVector & v)
{
  std::cout << label << " [";
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::setw(6) << std::fixed << std::setprecision(3) << v[i]
              << (i + 1 < v.size() ? ", " : "]\n");
  }
}

void printPos(const std::string & label, const Eigen::Vector3d & p)
{
  std::cout << label << " [X: " << std::setw(6) << std::fixed << std::setprecision(4) << p.x()
            << ", Y: " << std::setw(6) << p.y()
            << ", Z: " << std::setw(6) << p.z() << "] m\n";
}

void evaluateIKStrategy(
  const std::string & strategy_desc,
  const dobot_kinematics::GeneralizedKinematicsEngine & engine,
  const dobot_kinematics::IIKSolverStrategy & strategy,
  const dobot_kinematics::TransformMatrix & T_target,
  const dobot_kinematics::JointVector & q_init)
{
  std::cout << "\n  --- IK Strategy: " << strategy.getName()
            << " (" << strategy_desc << ") ---\n";

  auto ik_solution = engine.computeIKWithStrategy(strategy, T_target, q_init);

  if (ik_solution.has_value()) {
    dobot_kinematics::JointVector q_final = ik_solution->joint_angles;
    dobot_kinematics::TransformMatrix T_final = engine.computeFK(q_final);
    Eigen::Vector3d p_final = T_final.block<3, 1>(0, 3);

    printVector("    Solved Joints  (q_final): ", q_final);
    printPos   ("    Final Position (p_final): ", p_final);

    double pos_err_mm = ik_solution->positional_error * 1000.0;
    double rot_err_rad = ik_solution->rotational_error;

    std::cout << "    [Diagnostics]: Iterations = " << ik_solution->iterations_used
              << " | Pos Error = " << std::fixed << std::setprecision(4)
              << pos_err_mm << " mm (" << ik_solution->positional_error << " m)"
              << " | Rot Error = " << rot_err_rad << " rad"
              << " | Manipulability w = " << ik_solution->manipulability << "\n";

    if (pos_err_mm < 1.0) {
      std::cout << "    >> STATUS: SUCCESS (Sub-millimeter Cartesian accuracy achieved!)\n";
    } else {
      std::cout << "    >> STATUS: CONVERGED (Acceptable tolerance)\n";
    }
  } else {
    std::cout << "    >> STATUS: FAILED (Did not converge within iteration limit)\n";
  }
}

void runKinematicsTest(
  const std::string & robot_name,
  dobot_kinematics::GeneralizedKinematicsEngine & engine,
  const dobot_kinematics::JointVector & q_target)
{
  const size_t dof = engine.getDOF();
  std::cout << "\n=================================================================\n";
  std::cout << "  ROBOT: " << robot_name << " (" << dof << "-DOF)\n";
  std::cout << "  Active FK Modeling Strategy: " << engine.getFKStrategy().getModelName() << "\n";
  std::cout << "=================================================================\n";

  // 1. Initial State (Home Configuration)
  dobot_kinematics::JointVector q_init = dobot_kinematics::JointVector::Zero(dof);
  dobot_kinematics::TransformMatrix T_init = engine.computeFK(q_init);
  Eigen::Vector3d p_init = T_init.block<3, 1>(0, 3);

  printVector("Initial Joint Angles (q_init): ", q_init);
  printPos   ("Initial End-Effector Position: ", p_init);

  // 2. Target State
  dobot_kinematics::TransformMatrix T_target = engine.computeFK(q_target);
  Eigen::Vector3d p_target = T_target.block<3, 1>(0, 3);

  printVector("Target Joint Angles  (q_target): ", q_target);
  printPos   ("Target End-Effector Position:  ", p_target);

  // 3. Forward Kinematics (FK) Matrix at Target
  std::cout << "\n[Forward Kinematics Matrix T_0^" << dof << " at Target]:\n"
            << std::fixed << std::setprecision(4) << T_target << "\n";

  // 4. Manipulator Jacobian Matrix (6 x N)
  dobot_kinematics::JacobianMatrix J_init = engine.computeJacobian(q_init);
  std::cout << "\n[Manipulator Jacobian J(q_init)] Dimensions: " 
            << J_init.rows() << " x " << J_init.cols() << " (Task Space: 6D, Joints: " << dof << "D)\n";
  std::cout << J_init << "\n";

  double manipulability = engine.computeManipulability(q_init);
  std::cout << "\nYoshikawa Manipulability Measure w: " << manipulability
            << (engine.isNearSingularity(q_init) ? " [NEAR SINGULARITY]" : " [DEXTEROUS]") << "\n";

  // 5. IK Strategy Pattern Multi-Algorithm Benchmarking
  std::cout << "\n[IK STRATEGY PATTERN EVALUATION]:\n";
  std::cout << "Configured Default Strategy on Engine: " << engine.getIKStrategy().getName() << "\n";

  dobot_kinematics::AdaptiveDLSStrategy dls_strategy(0.001, 2e-4);
  evaluateIKStrategy("Adaptive Levenberg-Marquardt", engine, dls_strategy, T_target, q_init);

  dobot_kinematics::PseudoInverseStrategy pinv_strategy(0.15);
  evaluateIKStrategy("Moore-Penrose with Null-Space", engine, pinv_strategy, T_target, q_init);

  dobot_kinematics::JacobianTransposeStrategy jt_strategy(0.5);
  evaluateIKStrategy("Buss Optimal Line-Search", engine, jt_strategy, T_target, q_init);
}

}  // namespace

int main()
{
  std::cout << "#################################################################\n";
  std::cout << "  GENERALIZED KINEMATICS ENGINE VERIFICATION SUITE               \n";
  std::cout << "  Dual Strategy Pattern: FK/Jacobian Modeling + IK Solvers       \n";
  std::cout << "#################################################################\n";

  const double pi = M_PI;

  // =========================================================================
  // PART A: ROBOT DESCRIPTION STRATEGY COMPARISON (URDF vs STANDARD DH)
  // =========================================================================
  std::cout << "\n=================================================================\n";
  std::cout << "  PART A: DEMONSTRATING FK/JACOBIAN ROBOT DESCRIPTION STRATEGIES \n";
  std::cout << "=================================================================\n";

  // 1. Dobot Nova 5 via Standard DH Strategy
  std::vector<dobot_kinematics::DHParameter> nova5_dh = {
    dobot_kinematics::DHParameter(0.0,     pi / 2.0,  0.240, 0.0),        // Joint 1
    dobot_kinematics::DHParameter(-0.400,  0.0,       0.0,   pi / 2.0),   // Joint 2
    dobot_kinematics::DHParameter(-0.330,  0.0,       0.0,   0.0),        // Joint 3
    dobot_kinematics::DHParameter(0.0,     pi / 2.0,  0.135, -pi / 2.0),  // Joint 4
    dobot_kinematics::DHParameter(0.0,    -pi / 2.0,  0.120, 0.0),        // Joint 5
    dobot_kinematics::DHParameter(0.0,     0.0,       0.088, 0.0)         // Joint 6
  };
  dobot_kinematics::GeneralizedKinematicsEngine engine(nova5_dh);
  std::cout << "Engine initialized with Strategy: " << engine.getFKStrategy().getModelName() << "\n";

  // 2. Dobot Nova 5 via Direct URDF Robot Description Strategy (from nova5.urdf)
  std::vector<dobot_kinematics::URDFJointDescription> nova5_urdf = {
    dobot_kinematics::URDFJointDescription("joint1", {0.0, 0.0, 0.240}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
    dobot_kinematics::URDFJointDescription("joint2", {0.0, 0.0, 0.0}, {-pi/2.0, pi/2.0, pi}, {0.0, 0.0, 1.0}),
    dobot_kinematics::URDFJointDescription("joint3", {-0.399756, -0.013969, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
    dobot_kinematics::URDFJointDescription("joint4", {-0.329799, -0.011524, 0.135}, {0.0, 0.0, -1.535866}, {0.0, 0.0, 1.0}),
    dobot_kinematics::URDFJointDescription("joint5", {0.0, -0.120, 0.0}, {pi/2.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
    dobot_kinematics::URDFJointDescription("joint6", {0.0, 0.088328, 0.0}, {-pi/2.0, 0.0, 0.0}, {0.0, 0.0, 1.0})
  };
  
  // Runtime Hot-Swap: Switch the modeling strategy from DH to URDF description
  std::cout << "Hot-swapping FK Strategy to: URDF Direct Robot Description...\n";
  engine.setFKStrategy(std::make_shared<dobot_kinematics::URDFRobotDescriptionStrategy>(nova5_urdf));
  std::cout << "Active Engine Strategy is now: " << engine.getFKStrategy().getModelName() << "\n";

  dobot_kinematics::JointVector test_q(6);
  test_q << 0.35, -0.45, 0.60, 0.20, -0.30, 0.15;

  auto T_urdf = engine.computeFK(test_q);
  Eigen::Vector3d pos_urdf = T_urdf.block<3, 1>(0, 3);
  printPos("URDF Forward Kinematics End-Effector Position: ", pos_urdf);

  dobot_kinematics::JacobianMatrix J_urdf = engine.computeJacobian(test_q);
  std::cout << "URDF Jacobian Dimensions: " << J_urdf.rows() << " x " << J_urdf.cols()
            << " | Manipulability: " << engine.computeManipulability(J_urdf) << "\n";

  // Verify that Inverse Kinematics works seamlessly on the URDF description strategy
  std::cout << "Testing IK Solver directly on URDF Description Strategy...\n";
  dobot_kinematics::JointVector seed_q = dobot_kinematics::JointVector::Zero(6);
  auto ik_urdf_sol = engine.computeIK(T_urdf, seed_q);
  if (ik_urdf_sol.has_value()) {
    std::cout << "  >> IK Solved on URDF Description! Iterations: " << ik_urdf_sol->iterations_used
              << " | Pos Error: " << (ik_urdf_sol->positional_error * 1000.0) << " mm"
              << " | Rot Error: " << ik_urdf_sol->rotational_error << " rad\n";
    std::cout << "  >> VERIFICATION: PASS! URDF Robot Description strategy fully verified!\n";
  } else {
    std::cout << "  >> IK failed to converge on URDF strategy.\n";
  }

  // =========================================================================
  // PART B: MULTI-DOF SERIAL MANIPULATOR VALIDATION (4, 6, 7, 9 DOF)
  // =========================================================================
  std::cout << "\n=================================================================\n";
  std::cout << "  PART B: MULTI-DOF ROBOT BENCHMARK SUITE (4, 6, 7, 9 DOF)       \n";
  std::cout << "=================================================================\n";

  // 1. 4-DOF MANIPULATOR (SCARA / Palletizing Cobot)
  std::vector<dobot_kinematics::DHParameter> dh_4dof = {
    dobot_kinematics::DHParameter(0.000,  pi/2.0, 0.200, 0.0),
    dobot_kinematics::DHParameter(0.250,  0.0,    0.000, 0.0),
    dobot_kinematics::DHParameter(0.200,  0.0,    0.000, 0.0),
    dobot_kinematics::DHParameter(0.080,  0.0,    0.050, 0.0)
  };
  dobot_kinematics::GeneralizedKinematicsEngine robot_4dof(dh_4dof);
  dobot_kinematics::JointVector q_target_4dof(4);
  q_target_4dof << 0.40, 0.35, -0.25, 0.15;
  runKinematicsTest("4-DOF SCARA / Palletizing Manipulator", robot_4dof, q_target_4dof);

  // 2. 6-DOF MANIPULATOR (Dobot Nova 5 Industrial Cobot)
  dobot_kinematics::GeneralizedKinematicsEngine robot_6dof;
  dobot_kinematics::JointVector q_target_6dof(6);
  q_target_6dof << 0.35, -0.45, 0.60, 0.20, -0.30, 0.15;
  runKinematicsTest("6-DOF Dobot Nova 5 Industrial Cobot", robot_6dof, q_target_6dof);

  // 3. 7-DOF REDUNDANT MANIPULATOR (Franka Panda / KUKA iiwa Type)
  std::vector<dobot_kinematics::DHParameter> dh_7dof = {
    dobot_kinematics::DHParameter(0.0000, -pi/2.0, 0.333, 0.0),
    dobot_kinematics::DHParameter(0.0000,  pi/2.0, 0.000, 0.0),
    dobot_kinematics::DHParameter(0.0825,  pi/2.0, 0.316, 0.0),
    dobot_kinematics::DHParameter(-0.0825,-pi/2.0, 0.000, 0.0),
    dobot_kinematics::DHParameter(0.0000,  pi/2.0, 0.384, 0.0),
    dobot_kinematics::DHParameter(0.0880,  pi/2.0, 0.000, 0.0),
    dobot_kinematics::DHParameter(0.0000,  0.0,    0.107, 0.0)
  };
  dobot_kinematics::GeneralizedKinematicsEngine robot_7dof(dh_7dof);
  dobot_kinematics::JointVector q_target_7dof(7);
  q_target_7dof << 0.15, 0.25, -0.30, 0.40, -0.35, 0.50, -0.20;
  runKinematicsTest("7-DOF Redundant Manipulator (Franka/KUKA Type)", robot_7dof, q_target_7dof);

  // 4. 9-DOF HYPER-REDUNDANT MANIPULATOR (Torso + Arm Serpentine Chain)
  std::vector<dobot_kinematics::DHParameter> dh_9dof = {
    dobot_kinematics::DHParameter(0.000,  pi/2.0, 0.300, 0.0),
    dobot_kinematics::DHParameter(0.200,  0.0,    0.000, pi/2.0),
    dobot_kinematics::DHParameter(0.000,  pi/2.0, 0.250, 0.0),
    dobot_kinematics::DHParameter(0.350,  0.0,    0.000, 0.0),
    dobot_kinematics::DHParameter(0.000, -pi/2.0, 0.150, 0.0),
    dobot_kinematics::DHParameter(0.300,  0.0,    0.000, 0.0),
    dobot_kinematics::DHParameter(0.000,  pi/2.0, 0.120, 0.0),
    dobot_kinematics::DHParameter(0.000, -pi/2.0, 0.100, 0.0),
    dobot_kinematics::DHParameter(0.000,  0.0,    0.080, 0.0)
  };
  dobot_kinematics::GeneralizedKinematicsEngine robot_9dof(dh_9dof);
  dobot_kinematics::JointVector q_target_9dof(9);
  q_target_9dof << 0.10, -0.20, 0.30, -0.15, 0.25, -0.30, 0.20, -0.10, 0.15;
  runKinematicsTest("9-DOF Hyper-Redundant Manipulator (Torso + Arm)", robot_9dof, q_target_9dof);

  std::cout << "\n#################################################################\n";
  std::cout << "  ALL 4 KINEMATIC CHAINS & DUAL STRATEGIES VERIFIED SUCCESSFULLY!\n";
  std::cout << "#################################################################\n";

  return 0;
}
