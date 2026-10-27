#ifndef DOBOT_KINEMATICS__KINEMATICS_ENGINE_HPP_
#define DOBOT_KINEMATICS__KINEMATICS_ENGINE_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dobot_kinematics
{

class GeneralizedKinematicsEngine;

// ===========================================================================
// CORE TYPES & DATA STRUCTURES
// ===========================================================================

using JointVector = Eigen::VectorXd;                  ///< N x 1 joint angles vector
using TransformMatrix = Eigen::Matrix4d;             ///< 4 x 4 SE(3) homogeneous matrix
using JacobianMatrix = Eigen::MatrixXd;              ///< 6 x N geometric Jacobian
using TransformChain = std::vector<TransformMatrix>; ///< (N + 1) link transforms [T_0^0 ... T_0^N]

/**
 * @brief Standard Denavit-Hartenberg parameters with precomputed trigonometry
 */
struct DHParameter
{
  double a;             ///< Link length along x_i (meters)
  double alpha;         ///< Link twist about x_i (radians)
  double d;             ///< Link offset along z_{i-1} (meters)
  double theta_offset;  ///< Nominal angle offset about z_{i-1} (radians)
  double cos_alpha;     ///< Precomputed cos(alpha)
  double sin_alpha;     ///< Precomputed sin(alpha)

  DHParameter(double a_val, double alpha_val, double d_val, double offset_val)
  : a(a_val),
    alpha(alpha_val),
    d(d_val),
    theta_offset(offset_val),
    cos_alpha(std::cos(alpha_val)),
    sin_alpha(std::sin(alpha_val))
  {
  }
};

/**
 * @brief Modified Denavit-Hartenberg parameters (Craig convention)
 */
struct ModifiedDHParameter
{
  double alpha_prev;    ///< Link twist about x_{i-1} (radians)
  double a_prev;        ///< Link length along x_{i-1} (meters)
  double d;             ///< Link offset along z_i (meters)
  double theta_offset;  ///< Nominal angle offset about z_i (radians)
  double cos_alpha;     ///< Precomputed cos(alpha_{i-1})
  double sin_alpha;     ///< Precomputed sin(alpha_{i-1})

  ModifiedDHParameter(double alpha_val, double a_val, double d_val, double offset_val)
  : alpha_prev(alpha_val),
    a_prev(a_val),
    d(d_val),
    theta_offset(offset_val),
    cos_alpha(std::cos(alpha_val)),
    sin_alpha(std::sin(alpha_val))
  {
  }
};

/**
 * @brief URDF / Rigid Body Joint Description matching ROS 2 robot descriptions
 */
struct URDFJointDescription
{
  std::string name;
  Eigen::Vector3d origin_xyz{0.0, 0.0, 0.0};
  Eigen::Vector3d origin_rpy{0.0, 0.0, 0.0};  ///< roll (x), pitch (y), yaw (z)
  Eigen::Vector3d axis{0.0, 0.0, 1.0};        ///< Joint rotation/translation unit vector
  double lower_limit{-2.0 * M_PI};
  double upper_limit{ 2.0 * M_PI};
  bool is_revolute{true};                     ///< true: revolute, false: prismatic

  // Precomputed static transform T_origin = Translation(xyz) * Rotation(rpy)
  TransformMatrix T_origin{TransformMatrix::Identity()};

  URDFJointDescription(
    std::string joint_name,
    const Eigen::Vector3d & xyz,
    const Eigen::Vector3d & rpy,
    const Eigen::Vector3d & joint_axis = Eigen::Vector3d::UnitZ(),
    double min_lim = -2.0 * M_PI,
    double max_lim =  2.0 * M_PI,
    bool revolute = true)
  : name(std::move(joint_name)),
    origin_xyz(xyz),
    origin_rpy(rpy),
    axis(joint_axis.normalized()),
    lower_limit(min_lim),
    upper_limit(max_lim),
    is_revolute(revolute)
  {
    const Eigen::Matrix3d R = (
      Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
    ).toRotationMatrix();

    T_origin.setIdentity();
    T_origin.block<3, 3>(0, 0) = R;
    T_origin.block<3, 1>(0, 3) = xyz;
  }
};

/**
 * @brief Output result and diagnostic telemetry returned by an IK Strategy
 */
struct IKSolution
{
  JointVector joint_angles;  ///< Solved N-DOF joint angles (radians)
  double positional_error;   ///< Final Cartesian distance error (meters)
  double rotational_error;   ///< Final orientation angle error (radians)
  size_t iterations_used;    ///< Number of solver iterations
  double manipulability;     ///< Yoshikawa manipulability at solution pose
  std::string strategy_name; ///< Name of the strategy that generated the solution
  bool converged;            ///< True if target tolerance was met
};

// ===========================================================================
// STRATEGY PATTERN 1: FORWARD KINEMATICS & JACOBIAN MODELING STRATEGY
// ===========================================================================

/**
 * @brief Abstract Strategy for Robot Description & Forward Kinematics/Jacobian
 */
class IFKJacobianStrategy
{
public:
  virtual ~IFKJacobianStrategy() = default;

  /// Identifier for diagnostics
  virtual std::string getModelName() const noexcept = 0;

  /// Number of degrees of freedom
  virtual size_t getDOF() const noexcept = 0;

  /// Compute full chain of link frames T_0^0, T_0^1, ..., T_0^N
  virtual TransformChain computeChain(const JointVector & q) const = 0;

  /// Compute end-effector forward kinematics pose T_0^N
  virtual TransformMatrix computeFK(const JointVector & q) const = 0;

  /// Single-pass forward kinematics and 6xN geometric Jacobian computation
  virtual void computeFKAndJacobian(
    const JointVector & q,
    TransformMatrix & fk_out,
    JacobianMatrix & jacobian_out) const = 0;
};

// Concrete FK Strategy 1: Standard Denavit-Hartenberg (DH)
class StandardDHStrategy : public IFKJacobianStrategy
{
public:
  explicit StandardDHStrategy(std::vector<DHParameter> dh_table)
  : dh_table_(std::move(dh_table)) {}

  std::string getModelName() const noexcept override { return "Standard Denavit-Hartenberg (DH)"; }
  size_t getDOF() const noexcept override { return dh_table_.size(); }

  TransformChain computeChain(const JointVector & q) const override;
  TransformMatrix computeFK(const JointVector & q) const override;
  void computeFKAndJacobian(
    const JointVector & q,
    TransformMatrix & fk_out,
    JacobianMatrix & jacobian_out) const override;

  static TransformMatrix computeDHTransform(const DHParameter & param, double joint_angle) noexcept;
  const std::vector<DHParameter> & getParameters() const noexcept { return dh_table_; }

private:
  std::vector<DHParameter> dh_table_;
};

// Concrete FK Strategy 2: Modified Denavit-Hartenberg (Craig Convention)
class ModifiedDHStrategy : public IFKJacobianStrategy
{
public:
  explicit ModifiedDHStrategy(std::vector<ModifiedDHParameter> mdh_table)
  : mdh_table_(std::move(mdh_table)) {}

  std::string getModelName() const noexcept override { return "Modified Denavit-Hartenberg (Craig)"; }
  size_t getDOF() const noexcept override { return mdh_table_.size(); }

  TransformChain computeChain(const JointVector & q) const override;
  TransformMatrix computeFK(const JointVector & q) const override;
  void computeFKAndJacobian(
    const JointVector & q,
    TransformMatrix & fk_out,
    JacobianMatrix & jacobian_out) const override;

  static TransformMatrix computeMDHTransform(const ModifiedDHParameter & param, double joint_angle) noexcept;
  const std::vector<ModifiedDHParameter> & getParameters() const noexcept { return mdh_table_; }

private:
  std::vector<ModifiedDHParameter> mdh_table_;
};

// Concrete FK Strategy 3: Direct URDF Robot Description (Rigid Body Chain)
class URDFRobotDescriptionStrategy : public IFKJacobianStrategy
{
public:
  explicit URDFRobotDescriptionStrategy(std::vector<URDFJointDescription> joints)
  : joints_(std::move(joints)) {}

  std::string getModelName() const noexcept override { return "URDF Direct Robot Description"; }
  size_t getDOF() const noexcept override { return joints_.size(); }

  TransformChain computeChain(const JointVector & q) const override;
  TransformMatrix computeFK(const JointVector & q) const override;
  void computeFKAndJacobian(
    const JointVector & q,
    TransformMatrix & fk_out,
    JacobianMatrix & jacobian_out) const override;

  const std::vector<URDFJointDescription> & getJoints() const noexcept { return joints_; }

private:
  std::vector<URDFJointDescription> joints_;
};

// ===========================================================================
// STRATEGY PATTERN 2: INVERSE KINEMATICS SOLVER STRATEGY
// ===========================================================================

/**
 * @brief Abstract Strategy for Numerical Inverse Kinematics Solvers
 */
class IIKSolverStrategy
{
public:
  virtual ~IIKSolverStrategy() = default;

  /// Identifier for diagnostics
  virtual std::string getName() const noexcept = 0;

  virtual std::optional<IKSolution> solve(
    const GeneralizedKinematicsEngine & context,
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance,
    double rot_tolerance,
    size_t max_iterations) const = 0;
};

// Concrete IK Strategy 1: Adaptive DLS (Levenberg-Marquardt)
class AdaptiveDLSStrategy : public IIKSolverStrategy
{
public:
  explicit AdaptiveDLSStrategy(double base_damping = 0.001, double w_threshold = 2e-4)
  : base_damping_(base_damping), w_threshold_(w_threshold) {}

  std::string getName() const noexcept override { return "Adaptive Damped Least-Squares (Levenberg-Marquardt)"; }

  std::optional<IKSolution> solve(
    const GeneralizedKinematicsEngine & context,
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance,
    double rot_tolerance,
    size_t max_iterations) const override;

private:
  double base_damping_;
  double w_threshold_;
};

// Concrete IK Strategy 2: Moore-Penrose Pseudo-Inverse (with Null-Space Projection)
class PseudoInverseStrategy : public IIKSolverStrategy
{
public:
  explicit PseudoInverseStrategy(double null_space_gain = 0.15)
  : null_space_gain_(null_space_gain) {}

  std::string getName() const noexcept override { return "Moore-Penrose Pseudo-Inverse (with Null-Space Projection)"; }

  std::optional<IKSolution> solve(
    const GeneralizedKinematicsEngine & context,
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance,
    double rot_tolerance,
    size_t max_iterations) const override;

private:
  double null_space_gain_;
};

// Concrete IK Strategy 3: Jacobian Transpose (Gradient Descent)
class JacobianTransposeStrategy : public IIKSolverStrategy
{
public:
  explicit JacobianTransposeStrategy(double step_size = 0.5)
  : step_size_(step_size) {}

  std::string getName() const noexcept override { return "Jacobian Transpose (Buss Optimal Line-Search)"; }

  std::optional<IKSolution> solve(
    const GeneralizedKinematicsEngine & context,
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance,
    double rot_tolerance,
    size_t max_iterations) const override;

private:
  double step_size_;
};

// ===========================================================================
// CONTEXT: GENERALIZED KINEMATICS ENGINE
// ===========================================================================

class GeneralizedKinematicsEngine
{
public:
  using JointVector = dobot_kinematics::JointVector;
  using TransformMatrix = dobot_kinematics::TransformMatrix;
  using JacobianMatrix = dobot_kinematics::JacobianMatrix;
  using TransformChain = dobot_kinematics::TransformChain;
  using IKSolution = dobot_kinematics::IKSolution;

  /**
   * @brief Default constructor: Configures Dobot Nova 5 (6-DOF) with StandardDHStrategy & AdaptiveDLSStrategy
   */
  GeneralizedKinematicsEngine();

  /**
   * @brief Construct from Standard DH Parameter table
   */
  explicit GeneralizedKinematicsEngine(
    std::vector<DHParameter> dh_table,
    std::vector<double> joint_min = {},
    std::vector<double> joint_max = {},
    std::vector<std::string> joint_names = {},
    std::shared_ptr<IIKSolverStrategy> ik_strategy = nullptr);

  /**
   * @brief Construct from Modified DH Parameter table (Craig convention)
   */
  explicit GeneralizedKinematicsEngine(
    std::vector<ModifiedDHParameter> mdh_table,
    std::vector<double> joint_min = {},
    std::vector<double> joint_max = {},
    std::vector<std::string> joint_names = {},
    std::shared_ptr<IIKSolverStrategy> ik_strategy = nullptr);

  /**
   * @brief Construct from URDF Robot Description joint-link specifications
   */
  explicit GeneralizedKinematicsEngine(
    std::vector<URDFJointDescription> joints,
    std::shared_ptr<IIKSolverStrategy> ik_strategy = nullptr);

  /**
   * @brief Fully general constructor with injected FK and IK strategies
   */
  GeneralizedKinematicsEngine(
    std::shared_ptr<IFKJacobianStrategy> fk_strategy,
    std::vector<double> joint_min = {},
    std::vector<double> joint_max = {},
    std::vector<std::string> joint_names = {},
    std::shared_ptr<IIKSolverStrategy> ik_strategy = nullptr);

  // --- Strategy Management (Runtime Hot-Swapping) ---
  void setFKStrategy(std::shared_ptr<IFKJacobianStrategy> strategy) noexcept;
  const IFKJacobianStrategy & getFKStrategy() const noexcept { return *fk_strategy_; }

  void setIKStrategy(std::shared_ptr<IIKSolverStrategy> strategy) noexcept;
  const IIKSolverStrategy & getIKStrategy() const noexcept { return *ik_strategy_; }

  // --- 1. Forward Kinematics (FK) ---
  TransformChain computeChain(const JointVector & q) const;
  TransformMatrix computeFK(const JointVector & q) const;
  TransformMatrix computeFK(const std::vector<double> & q) const;

  // --- 2. Differential Kinematics & Manipulability ---
  void computeFKAndJacobian(const JointVector & q, TransformMatrix & fk_out, JacobianMatrix & jacobian_out) const;
  JacobianMatrix computeJacobian(const JointVector & q) const;
  JacobianMatrix computeJacobian(const std::vector<double> & q) const;
  double computeManipulability(const JacobianMatrix & J) const noexcept;
  double computeManipulability(const JointVector & q) const;
  bool isNearSingularity(const JointVector & q, double threshold = 1e-4) const;

  // --- 3. Inverse Kinematics (IK) ---
  std::optional<IKSolution> computeIK(
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance = 1e-4,
    double rot_tolerance = 1e-3,
    size_t max_iterations = 250) const;

  std::optional<IKSolution> computeIKWithStrategy(
    const IIKSolverStrategy & strategy,
    const TransformMatrix & target_pose,
    const JointVector & seed_q,
    double pos_tolerance = 1e-4,
    double rot_tolerance = 1e-3,
    size_t max_iterations = 250) const;

  std::optional<std::vector<double>> computeIK(
    const TransformMatrix & target_pose,
    const std::vector<double> & seed_q) const;

  // --- 4. Utilities & Limits ---
  size_t getDOF() const noexcept;
  JointVector clampJoints(const JointVector & q) const;
  bool isWithinLimits(const JointVector & q) const;
  JointVector computeJointCenteringGradient(const JointVector & q) const;

  const JointVector & getJointMin() const noexcept { return joint_min_; }
  const JointVector & getJointMax() const noexcept { return joint_max_; }
  const std::vector<std::string> & getJointNames() const noexcept { return joint_names_; }

private:
  void initLimits(const std::vector<double> & joint_min, const std::vector<double> & joint_max);

  std::shared_ptr<IFKJacobianStrategy> fk_strategy_;
  std::shared_ptr<IIKSolverStrategy> ik_strategy_;

  size_t num_joints_{0};
  std::vector<std::string> joint_names_;
  JointVector joint_min_;
  JointVector joint_max_;
  JointVector joint_center_;
  JointVector joint_range_sq_;
};

// Aliases for naming flexibility and backward compatibility
using GeneralizedKinematics_Engine = GeneralizedKinematicsEngine;
using KinematicsSolver = GeneralizedKinematicsEngine;

}  // namespace dobot_kinematics

#endif  // DOBOT_KINEMATICS__KINEMATICS_ENGINE_HPP_

