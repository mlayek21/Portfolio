#include "dobot_kinematics/kinematics_engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace dobot_kinematics
{

namespace
{
JointVector toJointVector(const std::vector<double> & vec, size_t n)
{
  JointVector q = JointVector::Zero(n);
  for (size_t i = 0; i < n && i < vec.size(); ++i) {
    q[i] = vec[i];
  }
  return q;
}

std::vector<double> toStdVector(const JointVector & q)
{
  return std::vector<double>(q.data(), q.data() + q.size());
}

struct PoseError
{
  Eigen::Matrix<double, 6, 1> error_vector;
  double pos_error_norm;
  double rot_error_norm;
};

PoseError computePoseError(
  const TransformMatrix & T_target,
  const TransformMatrix & T_current)
{
  const Eigen::Vector3d p_target = T_target.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_target = T_target.block<3, 3>(0, 0);

  const Eigen::Vector3d p_current = T_current.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_current = T_current.block<3, 3>(0, 0);

  const Eigen::Vector3d e_pos = p_target - p_current;

  // Orientation error via R_target * R_current^T
  const Eigen::Matrix3d R_err = R_target * R_current.transpose();
  const Eigen::AngleAxisd angle_axis(R_err);
  const double angle = angle_axis.angle();
  Eigen::Vector3d e_rot = Eigen::Vector3d::Zero();
  if (std::abs(angle) > 1e-9) {
    e_rot = angle_axis.axis() * angle;
  }

  PoseError pe;
  pe.error_vector.block<3, 1>(0, 0) = e_pos;
  pe.error_vector.block<3, 1>(3, 0) = e_rot;
  pe.pos_error_norm = e_pos.norm();
  pe.rot_error_norm = e_rot.norm();
  return pe;
}
}  // namespace

// ===========================================================================
// FK STRATEGY 1: STANDARD DENAVIT-HARTENBERG (DH)
// ===========================================================================

TransformMatrix StandardDHStrategy::computeDHTransform(
  const DHParameter & param,
  double joint_angle) noexcept
{
  const double theta = joint_angle + param.theta_offset;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = param.cos_alpha;
  const double sa = param.sin_alpha;

  TransformMatrix T;
  T << ct, -st * ca,  st * sa, param.a * ct,
       st,  ct * ca, -ct * sa, param.a * st,
      0.0,       sa,       ca, param.d,
      0.0,      0.0,      0.0, 1.0;
  return T;
}

TransformChain StandardDHStrategy::computeChain(const JointVector & q) const
{
  TransformChain chain;
  chain.reserve(dh_table_.size() + 1);
  chain.push_back(TransformMatrix::Identity());

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < dh_table_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeDHTransform(dh_table_[i], angle);
    chain.push_back(current_T);
  }
  return chain;
}

TransformMatrix StandardDHStrategy::computeFK(const JointVector & q) const
{
  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < dh_table_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeDHTransform(dh_table_[i], angle);
  }
  return current_T;
}

void StandardDHStrategy::computeFKAndJacobian(
  const JointVector & q,
  TransformMatrix & fk_out,
  JacobianMatrix & jacobian_out) const
{
  const size_t n = dh_table_.size();
  std::array<Eigen::Vector3d, 16> p_stack;
  std::vector<Eigen::Vector3d> p_heap;
  Eigen::Vector3d * p_origins = p_stack.data();
  if (n > 16) {
    p_heap.resize(n);
    p_origins = p_heap.data();
  }

  jacobian_out.resize(6, n);

  Eigen::Vector3d current_z = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d current_p = Eigen::Vector3d::Zero();

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < n; ++i) {
    jacobian_out.block<3, 1>(3, i) = current_z;
    p_origins[i] = current_p;

    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeDHTransform(dh_table_[i], angle);

    current_z = current_T.block<3, 1>(0, 2);
    current_p = current_T.block<3, 1>(0, 3);
  }

  fk_out = current_T;
  const Eigen::Vector3d p_e = fk_out.block<3, 1>(0, 3);

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d & z_i = jacobian_out.block<3, 1>(3, i);
    jacobian_out.block<3, 1>(0, i).noalias() = z_i.cross(p_e - p_origins[i]);
  }
}

// ===========================================================================
// FK STRATEGY 2: MODIFIED DENAVIT-HARTENBERG (CRAIG CONVENTION)
// ===========================================================================

TransformMatrix ModifiedDHStrategy::computeMDHTransform(
  const ModifiedDHParameter & param,
  double joint_angle) noexcept
{
  const double theta = joint_angle + param.theta_offset;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = param.cos_alpha;
  const double sa = param.sin_alpha;

  TransformMatrix T;
  T << ct,       -st,       0.0, param.a_prev,
       st * ca,   ct * ca,  -sa, -param.d * sa,
       st * sa,   ct * sa,   ca,  param.d * ca,
       0.0,       0.0,      0.0, 1.0;
  return T;
}

TransformChain ModifiedDHStrategy::computeChain(const JointVector & q) const
{
  TransformChain chain;
  chain.reserve(mdh_table_.size() + 1);
  chain.push_back(TransformMatrix::Identity());

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < mdh_table_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeMDHTransform(mdh_table_[i], angle);
    chain.push_back(current_T);
  }
  return chain;
}

TransformMatrix ModifiedDHStrategy::computeFK(const JointVector & q) const
{
  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < mdh_table_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeMDHTransform(mdh_table_[i], angle);
  }
  return current_T;
}

void ModifiedDHStrategy::computeFKAndJacobian(
  const JointVector & q,
  TransformMatrix & fk_out,
  JacobianMatrix & jacobian_out) const
{
  const size_t n = mdh_table_.size();
  std::array<Eigen::Vector3d, 16> p_stack;
  std::vector<Eigen::Vector3d> p_heap;
  Eigen::Vector3d * p_origins = p_stack.data();
  if (n > 16) {
    p_heap.resize(n);
    p_origins = p_heap.data();
  }

  jacobian_out.resize(6, n);

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < n; ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    current_T = current_T * computeMDHTransform(mdh_table_[i], angle);

    jacobian_out.block<3, 1>(3, i) = current_T.block<3, 1>(0, 2);
    p_origins[i] = current_T.block<3, 1>(0, 3);
  }

  fk_out = current_T;
  const Eigen::Vector3d p_e = fk_out.block<3, 1>(0, 3);

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d & z_i = jacobian_out.block<3, 1>(3, i);
    jacobian_out.block<3, 1>(0, i).noalias() = z_i.cross(p_e - p_origins[i]);
  }
}

// ===========================================================================
// FK STRATEGY 3: DIRECT URDF ROBOT DESCRIPTION (RIGID BODY CHAIN)
// ===========================================================================

TransformChain URDFRobotDescriptionStrategy::computeChain(const JointVector & q) const
{
  TransformChain chain;
  chain.reserve(joints_.size() + 1);
  chain.push_back(TransformMatrix::Identity());

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    const auto & joint = joints_[i];

    TransformMatrix T_joint = TransformMatrix::Identity();
    if (joint.is_revolute) {
      T_joint.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
    } else {
      T_joint.block<3, 1>(0, 3) = angle * joint.axis;
    }

    current_T = current_T * (joint.T_origin * T_joint);
    chain.push_back(current_T);
  }

  return chain;
}

TransformMatrix URDFRobotDescriptionStrategy::computeFK(const JointVector & q) const
{
  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    const auto & joint = joints_[i];

    TransformMatrix T_joint = TransformMatrix::Identity();
    if (joint.is_revolute) {
      T_joint.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
    } else {
      T_joint.block<3, 1>(0, 3) = angle * joint.axis;
    }

    current_T = current_T * (joint.T_origin * T_joint);
  }
  return current_T;
}

void URDFRobotDescriptionStrategy::computeFKAndJacobian(
  const JointVector & q,
  TransformMatrix & fk_out,
  JacobianMatrix & jacobian_out) const
{
  const size_t n = joints_.size();
  std::array<Eigen::Vector3d, 16> p_stack;
  std::vector<Eigen::Vector3d> p_heap;
  Eigen::Vector3d * p_origins = p_stack.data();
  if (n > 16) {
    p_heap.resize(n);
    p_origins = p_heap.data();
  }

  jacobian_out.resize(6, n);

  TransformMatrix current_T = TransformMatrix::Identity();
  for (size_t i = 0; i < n; ++i) {
    const double angle = (i < static_cast<size_t>(q.size())) ? q[i] : 0.0;
    const auto & joint = joints_[i];

    // Frame after parent link transform and joint origin offset
    TransformMatrix T_frame = current_T * joint.T_origin;
    jacobian_out.block<3, 1>(3, i) = T_frame.block<3, 3>(0, 0) * joint.axis;
    p_origins[i] = T_frame.block<3, 1>(0, 3);

    TransformMatrix T_joint = TransformMatrix::Identity();
    if (joint.is_revolute) {
      T_joint.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
    } else {
      T_joint.block<3, 1>(0, 3) = angle * joint.axis;
    }

    current_T = T_frame * T_joint;
  }

  fk_out = current_T;
  const Eigen::Vector3d p_e = fk_out.block<3, 1>(0, 3);

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d & z_i = jacobian_out.block<3, 1>(3, i);
    if (joints_[i].is_revolute) {
      jacobian_out.block<3, 1>(0, i).noalias() = z_i.cross(p_e - p_origins[i]);
    } else {
      jacobian_out.block<3, 1>(0, i) = z_i;
      jacobian_out.block<3, 1>(3, i).setZero();
    }
  }
}

// ===========================================================================
// IK STRATEGY 1: ADAPTIVE DAMPED LEAST SQUARES (LEVENBERG-MARQUARDT)
// ===========================================================================

std::optional<IKSolution> AdaptiveDLSStrategy::solve(
  const GeneralizedKinematicsEngine & context,
  const TransformMatrix & target_pose,
  const JointVector & seed_q,
  double pos_tolerance,
  double rot_tolerance,
  size_t max_iterations) const
{
  const size_t dof = context.getDOF();
  JointVector q = context.clampJoints(seed_q);
  if (static_cast<size_t>(q.size()) != dof) {
    q = JointVector::Zero(dof);
  }

  TransformMatrix current_T;
  JacobianMatrix J(6, dof);
  JointVector delta_q(dof);

  for (size_t iter = 0; iter < max_iterations; ++iter) {
    // Single-pass FK and Jacobian extraction via context's active FK strategy
    context.computeFKAndJacobian(q, current_T, J);

    const PoseError pe = computePoseError(target_pose, current_T);

    if (pe.pos_error_norm < pos_tolerance && pe.rot_error_norm < rot_tolerance) {
      const double manipulability = context.computeManipulability(J);
      return IKSolution{
        q, pe.pos_error_norm, pe.rot_error_norm, iter + 1,
        manipulability, getName(), true};
    }

    // Adaptive Levenberg-Marquardt damping lambda(w)
    const double w = context.computeManipulability(J);
    double lambda = base_damping_;
    if (w < w_threshold_) {
      const double ratio = (w_threshold_ - w) / w_threshold_;
      lambda += 0.08 * (ratio * ratio);
    }

    // Eigen .noalias() register accumulation
    Eigen::Matrix<double, 6, 6> JJt;
    JJt.noalias() = J * J.transpose();
    const Eigen::Matrix<double, 6, 6> damped =
      JJt + (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();

    // Minimum-norm joint update via Cholesky LDLT
    delta_q.noalias() = J.transpose() * damped.ldlt().solve(pe.error_vector);

    // Step clamp to prevent divergent leaps
    const double step_norm = delta_q.norm();
    if (step_norm > 0.25) {
      delta_q *= (0.25 / step_norm);
    }

    q += delta_q;
    q = context.clampJoints(q);
  }

  return std::nullopt;
}

// ===========================================================================
// IK STRATEGY 2: MOORE-PENROSE PSEUDO-INVERSE (WITH NULL-SPACE PROJECTION)
// ===========================================================================

std::optional<IKSolution> PseudoInverseStrategy::solve(
  const GeneralizedKinematicsEngine & context,
  const TransformMatrix & target_pose,
  const JointVector & seed_q,
  double pos_tolerance,
  double rot_tolerance,
  size_t max_iterations) const
{
  const size_t dof = context.getDOF();
  JointVector q = context.clampJoints(seed_q);
  if (static_cast<size_t>(q.size()) != dof) {
    q = JointVector::Zero(dof);
  }

  TransformMatrix current_T;
  JacobianMatrix J(6, dof);
  JointVector delta_q(dof);

  for (size_t iter = 0; iter < max_iterations; ++iter) {
    context.computeFKAndJacobian(q, current_T, J);

    const PoseError pe = computePoseError(target_pose, current_T);

    if (pe.pos_error_norm < pos_tolerance && pe.rot_error_norm < rot_tolerance) {
      const double manipulability = context.computeManipulability(J);
      return IKSolution{
        q, pe.pos_error_norm, pe.rot_error_norm, iter + 1,
        manipulability, getName(), true};
    }

    // Moore-Penrose Pseudo-Inverse via Complete Orthogonal Decomposition
    auto cod = J.completeOrthogonalDecomposition();
    delta_q.noalias() = cod.solve(pe.error_vector);

    // Null-Space Projection for Redundant Manipulators (dof > 6)
    if (dof > 6) {
      Eigen::MatrixXd I_N = Eigen::MatrixXd::Identity(dof, dof);
      Eigen::MatrixXd P_null = I_N - cod.pseudoInverse() * J;
      JointVector grad_H = context.computeJointCenteringGradient(q);
      delta_q.noalias() -= null_space_gain_ * (P_null * grad_H);
    }

    const double step_norm = delta_q.norm();
    if (step_norm > 0.25) {
      delta_q *= (0.25 / step_norm);
    }

    q += delta_q;
    q = context.clampJoints(q);
  }

  return std::nullopt;
}

// ===========================================================================
// IK STRATEGY 3: JACOBIAN TRANSPOSE (BUSS OPTIMAL LINE-SEARCH)
// ===========================================================================

std::optional<IKSolution> JacobianTransposeStrategy::solve(
  const GeneralizedKinematicsEngine & context,
  const TransformMatrix & target_pose,
  const JointVector & seed_q,
  double pos_tolerance,
  double rot_tolerance,
  size_t max_iterations) const
{
  const size_t dof = context.getDOF();
  JointVector q = context.clampJoints(seed_q);
  if (static_cast<size_t>(q.size()) != dof) {
    q = JointVector::Zero(dof);
  }

  TransformMatrix current_T;
  JacobianMatrix J(6, dof);
  JointVector delta_q(dof);

  for (size_t iter = 0; iter < max_iterations; ++iter) {
    context.computeFKAndJacobian(q, current_T, J);

    const PoseError pe = computePoseError(target_pose, current_T);

    if (pe.pos_error_norm < pos_tolerance && pe.rot_error_norm < rot_tolerance) {
      const double manipulability = context.computeManipulability(J);
      return IKSolution{
        q, pe.pos_error_norm, pe.rot_error_norm, iter + 1,
        manipulability, getName(), true};
    }

    // Optimal step size for Jacobian Transpose (Buss & Welman line-search formula)
    const JointVector Jt_e = J.transpose() * pe.error_vector;
    const Eigen::Matrix<double, 6, 1> JJt_e = J * Jt_e;
    const double denom = JJt_e.squaredNorm();
    double alpha = step_size_;
    if (denom > 1e-12) {
      alpha = pe.error_vector.dot(JJt_e) / denom;
    }
    delta_q.noalias() = alpha * Jt_e;

    const double step_norm = delta_q.norm();
    if (step_norm > 0.25) {
      delta_q *= (0.25 / step_norm);
    }

    q += delta_q;
    q = context.clampJoints(q);
  }

  return std::nullopt;
}

// ===========================================================================
// CONTEXT IMPLEMENTATION: GENERALIZED KINEMATICS ENGINE
// ===========================================================================

GeneralizedKinematicsEngine::GeneralizedKinematicsEngine()
{
  const double pi = M_PI;

  // Dobot Nova 5 (6-DOF) exact URDF joint descriptions matching nova5.urdf (ROS 2 base_link frame)
  std::vector<URDFJointDescription> urdf_joints = {
    URDFJointDescription(
      "joint1",
      Eigen::Vector3d(0.0, 0.0, 0.240),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)),
    URDFJointDescription(
      "joint2",
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(-1.57080287682252, 1.53586622836832, 3.14159265358979),
      Eigen::Vector3d(0.0, 0.0, 1.0)),
    URDFJointDescription(
      "joint3",
      Eigen::Vector3d(-0.399756009268664, -0.0139690033141953, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)),
    URDFJointDescription(
      "joint4",
      Eigen::Vector3d(-0.329798707647103, -0.0115244277211674, 0.134999532858734),
      Eigen::Vector3d(0.0, 0.0, -1.53586622840712),
      Eigen::Vector3d(0.0, 0.0, 1.0)),
    URDFJointDescription(
      "joint5",
      Eigen::Vector3d(0.0, -0.12, 0.0),
      Eigen::Vector3d(1.5708, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)),
    URDFJointDescription(
      "joint6",
      Eigen::Vector3d(0.0, 0.088328, 0.0),
      Eigen::Vector3d(-1.5708, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0))
  };

  fk_strategy_ = std::make_shared<URDFRobotDescriptionStrategy>(std::move(urdf_joints));
  num_joints_ = fk_strategy_->getDOF();
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  std::vector<double> j_min = {-2.0 * pi, -pi, -2.79, -2.0 * pi, -2.0 * pi, -2.0 * pi};
  std::vector<double> j_max = { 2.0 * pi,  pi,  2.79,  2.0 * pi,  2.0 * pi,  2.0 * pi};
  initLimits(j_min, j_max);

  ik_strategy_ = std::make_shared<AdaptiveDLSStrategy>();
}

GeneralizedKinematicsEngine::GeneralizedKinematicsEngine(
  std::vector<DHParameter> dh_table,
  std::vector<double> joint_min,
  std::vector<double> joint_max,
  std::vector<std::string> joint_names,
  std::shared_ptr<IIKSolverStrategy> ik_strategy)
: fk_strategy_(std::make_shared<StandardDHStrategy>(std::move(dh_table))),
  ik_strategy_(std::move(ik_strategy)),
  joint_names_(std::move(joint_names))
{
  num_joints_ = fk_strategy_->getDOF();
  initLimits(joint_min, joint_max);

  if (!ik_strategy_) {
    if (num_joints_ > 6) {
      ik_strategy_ = std::make_shared<PseudoInverseStrategy>();
    } else {
      ik_strategy_ = std::make_shared<AdaptiveDLSStrategy>();
    }
  }
}

GeneralizedKinematicsEngine::GeneralizedKinematicsEngine(
  std::vector<ModifiedDHParameter> mdh_table,
  std::vector<double> joint_min,
  std::vector<double> joint_max,
  std::vector<std::string> joint_names,
  std::shared_ptr<IIKSolverStrategy> ik_strategy)
: fk_strategy_(std::make_shared<ModifiedDHStrategy>(std::move(mdh_table))),
  ik_strategy_(std::move(ik_strategy)),
  joint_names_(std::move(joint_names))
{
  num_joints_ = fk_strategy_->getDOF();
  initLimits(joint_min, joint_max);

  if (!ik_strategy_) {
    if (num_joints_ > 6) {
      ik_strategy_ = std::make_shared<PseudoInverseStrategy>();
    } else {
      ik_strategy_ = std::make_shared<AdaptiveDLSStrategy>();
    }
  }
}

GeneralizedKinematicsEngine::GeneralizedKinematicsEngine(
  std::vector<URDFJointDescription> joints,
  std::shared_ptr<IIKSolverStrategy> ik_strategy)
: ik_strategy_(std::move(ik_strategy))
{
  num_joints_ = joints.size();
  std::vector<double> j_min(num_joints_);
  std::vector<double> j_max(num_joints_);
  joint_names_.resize(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i) {
    joint_names_[i] = joints[i].name;
    j_min[i] = joints[i].lower_limit;
    j_max[i] = joints[i].upper_limit;
  }

  fk_strategy_ = std::make_shared<URDFRobotDescriptionStrategy>(std::move(joints));
  initLimits(j_min, j_max);

  if (!ik_strategy_) {
    if (num_joints_ > 6) {
      ik_strategy_ = std::make_shared<PseudoInverseStrategy>();
    } else {
      ik_strategy_ = std::make_shared<AdaptiveDLSStrategy>();
    }
  }
}

GeneralizedKinematicsEngine::GeneralizedKinematicsEngine(
  std::shared_ptr<IFKJacobianStrategy> fk_strategy,
  std::vector<double> joint_min,
  std::vector<double> joint_max,
  std::vector<std::string> joint_names,
  std::shared_ptr<IIKSolverStrategy> ik_strategy)
: fk_strategy_(std::move(fk_strategy)),
  ik_strategy_(std::move(ik_strategy)),
  joint_names_(std::move(joint_names))
{
  num_joints_ = fk_strategy_ ? fk_strategy_->getDOF() : 0;
  initLimits(joint_min, joint_max);

  if (!ik_strategy_) {
    if (num_joints_ > 6) {
      ik_strategy_ = std::make_shared<PseudoInverseStrategy>();
    } else {
      ik_strategy_ = std::make_shared<AdaptiveDLSStrategy>();
    }
  }
}

void GeneralizedKinematicsEngine::initLimits(
  const std::vector<double> & joint_min,
  const std::vector<double> & joint_max)
{
  const double pi = M_PI;
  joint_min_.resize(num_joints_);
  joint_max_.resize(num_joints_);
  joint_center_.resize(num_joints_);
  joint_range_sq_.resize(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i) {
    joint_min_[i] = (i < joint_min.size()) ? joint_min[i] : -2.0 * pi;
    joint_max_[i] = (i < joint_max.size()) ? joint_max[i] :  2.0 * pi;
    joint_center_[i] = 0.5 * (joint_max_[i] + joint_min_[i]);
    const double range = joint_max_[i] - joint_min_[i];
    joint_range_sq_[i] = range * range;
  }

  if (joint_names_.size() < num_joints_) {
    joint_names_.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      if (joint_names_[i].empty()) {
        joint_names_[i] = "joint" + std::to_string(i + 1);
      }
    }
  }
}

void GeneralizedKinematicsEngine::setFKStrategy(std::shared_ptr<IFKJacobianStrategy> strategy) noexcept
{
  if (strategy) {
    fk_strategy_ = std::move(strategy);
    num_joints_ = fk_strategy_->getDOF();
  }
}

void GeneralizedKinematicsEngine::setIKStrategy(std::shared_ptr<IIKSolverStrategy> strategy) noexcept
{
  if (strategy) {
    ik_strategy_ = std::move(strategy);
  }
}

TransformChain GeneralizedKinematicsEngine::computeChain(const JointVector & q) const
{
  return fk_strategy_->computeChain(q);
}

TransformMatrix GeneralizedKinematicsEngine::computeFK(const JointVector & q) const
{
  return fk_strategy_->computeFK(q);
}

TransformMatrix GeneralizedKinematicsEngine::computeFK(const std::vector<double> & q) const
{
  return computeFK(toJointVector(q, num_joints_));
}

void GeneralizedKinematicsEngine::computeFKAndJacobian(
  const JointVector & q,
  TransformMatrix & fk_out,
  JacobianMatrix & jacobian_out) const
{
  fk_strategy_->computeFKAndJacobian(q, fk_out, jacobian_out);
}

JacobianMatrix GeneralizedKinematicsEngine::computeJacobian(const JointVector & q) const
{
  TransformMatrix dummy_T;
  JacobianMatrix J;
  computeFKAndJacobian(q, dummy_T, J);
  return J;
}

JacobianMatrix GeneralizedKinematicsEngine::computeJacobian(const std::vector<double> & q) const
{
  return computeJacobian(toJointVector(q, num_joints_));
}

double GeneralizedKinematicsEngine::computeManipulability(const JacobianMatrix & J) const noexcept
{
  Eigen::Matrix<double, 6, 6> JJt;
  JJt.noalias() = J * J.transpose();
  const double det = JJt.determinant();
  return (det > 0.0) ? std::sqrt(det) : 0.0;
}

double GeneralizedKinematicsEngine::computeManipulability(const JointVector & q) const
{
  return computeManipulability(computeJacobian(q));
}

bool GeneralizedKinematicsEngine::isNearSingularity(const JointVector & q, double threshold) const
{
  return computeManipulability(q) < threshold;
}

std::optional<IKSolution> GeneralizedKinematicsEngine::computeIK(
  const TransformMatrix & target_pose,
  const JointVector & seed_q,
  double pos_tolerance,
  double rot_tolerance,
  size_t max_iterations) const
{
  if (!ik_strategy_) {
    return std::nullopt;
  }
  return ik_strategy_->solve(
    *this, target_pose, seed_q, pos_tolerance, rot_tolerance, max_iterations);
}

std::optional<IKSolution> GeneralizedKinematicsEngine::computeIKWithStrategy(
  const IIKSolverStrategy & strategy,
  const TransformMatrix & target_pose,
  const JointVector & seed_q,
  double pos_tolerance,
  double rot_tolerance,
  size_t max_iterations) const
{
  return strategy.solve(
    *this, target_pose, seed_q, pos_tolerance, rot_tolerance, max_iterations);
}

std::optional<std::vector<double>> GeneralizedKinematicsEngine::computeIK(
  const TransformMatrix & target_pose,
  const std::vector<double> & seed_q) const
{
  const auto sol = computeIK(target_pose, toJointVector(seed_q, num_joints_));
  if (sol.has_value()) {
    return toStdVector(sol->joint_angles);
  }
  return std::nullopt;
}

size_t GeneralizedKinematicsEngine::getDOF() const noexcept
{
  return num_joints_;
}

JointVector GeneralizedKinematicsEngine::clampJoints(const JointVector & q) const
{
  JointVector clamped = q;
  if (static_cast<size_t>(clamped.size()) != num_joints_) {
    clamped = JointVector::Zero(num_joints_);
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    clamped[i] = std::clamp(clamped[i], joint_min_[i], joint_max_[i]);
  }
  return clamped;
}

bool GeneralizedKinematicsEngine::isWithinLimits(const JointVector & q) const
{
  if (static_cast<size_t>(q.size()) != num_joints_) {
    return false;
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    if (q[i] < joint_min_[i] || q[i] > joint_max_[i]) {
      return false;
    }
  }
  return true;
}

JointVector GeneralizedKinematicsEngine::computeJointCenteringGradient(const JointVector & q) const
{
  JointVector grad = JointVector::Zero(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    if (joint_range_sq_[i] > 1e-6) {
      grad[i] = 2.0 * (q[i] - joint_center_[i]) / joint_range_sq_[i];
    }
  }
  return grad;
}

}  // namespace dobot_kinematics
