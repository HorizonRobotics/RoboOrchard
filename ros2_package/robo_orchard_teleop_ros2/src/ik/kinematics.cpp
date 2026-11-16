// Project RoboOrchard
// Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-prismatic-unaligned.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/multibody/model.hpp>
#include <stdexcept>
#include <unordered_set>

#include "ik.hpp"

struct Kinematics::Impl {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::SE3 tail = pinocchio::SE3::Identity(), tip;
  pinocchio::JointIndex last = 0;
  std::vector<std::string> names;
  Q lo, hi, last_q;
  void update(const Q& q) {
    if (q.size() != model.nq || !q.allFinite())
      throw std::invalid_argument("Invalid joint configuration");
    if ((q.array() == last_q.array()).all()) return;
    pinocchio::computeJointJacobians(model, *data, q);
    tip = data->oMi[last] * tail;
    last_q = q;
  }
};

Kinematics::Kinematics(const std::string& path, const std::string& base,
                       const std::string& ee)
    : impl_(new Impl) {
  auto urdf = urdf::parseURDFFile(path);
  if (!urdf) throw std::invalid_argument("Unable to parse URDF");
  if (!urdf->getLink(base))
    throw std::invalid_argument("URDF base link does not exist: " + base);
  if (!urdf->getLink(ee))
    throw std::invalid_argument("URDF end-effector link does not exist: " + ee);
  if (base == ee)
    throw std::invalid_argument("Base and end-effector links must differ");
  std::vector<urdf::JointConstSharedPtr> joints;
  std::unordered_set<std::string> visited;
  auto link = urdf->getLink(ee);
  while (link->name != base) {
    if (!visited.insert(link->name).second)
      throw std::invalid_argument("Cycle in selected URDF chain");
    auto joint = link->parent_joint;
    if (!joint)
      throw std::invalid_argument("End-effector is not connected to base");
    joints.push_back(joint);
    link = urdf->getLink(joint->parent_link_name);
  }
  std::reverse(joints.begin(), joints.end());
  pinocchio::SE3 pending = pinocchio::SE3::Identity();
  std::vector<double> lower, upper;
  for (const auto& joint : joints) {
    const auto& origin = joint->parent_to_joint_origin_transform;
    // URDF uses Rz(yaw)*Ry(pitch)*Rx(roll); keep parsed decimal precision.
    Eigen::Quaterniond rotation(origin.rotation.w, origin.rotation.x,
                                origin.rotation.y, origin.rotation.z);
    pending = pending * pinocchio::SE3(rotation.toRotationMatrix(),
                                       Eigen::Vector3d(origin.position.x,
                                                       origin.position.y,
                                                       origin.position.z));
    if (joint->type == urdf::Joint::FIXED) continue;
    if (joint->mimic)
      throw std::invalid_argument("Mimic joints are not supported on IK chain");
    Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
    if (!axis.allFinite() || axis.norm() == 0)
      throw std::invalid_argument("Invalid joint axis: " + joint->name);
    axis.normalize();
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::CONTINUOUS) {
      impl_->last = impl_->model.addJoint(
          impl_->last, pinocchio::JointModelRevoluteUnaligned(axis), pending,
          joint->name);
    } else if (joint->type == urdf::Joint::PRISMATIC) {
      impl_->last = impl_->model.addJoint(
          impl_->last, pinocchio::JointModelPrismaticUnaligned(axis), pending,
          joint->name);
    } else {
      throw std::invalid_argument("Unsupported joint type: " + joint->name);
    }
    double lo = -std::numeric_limits<double>::infinity();
    double hi = std::numeric_limits<double>::infinity();
    if (joint->type != urdf::Joint::CONTINUOUS && joint->limits) {
      lo = joint->limits->lower;
      hi = joint->limits->upper;
    }
    if (!(lo < hi))
      throw std::invalid_argument(
          "Joint lower limit must be below upper limit");
    lower.push_back(lo);
    upper.push_back(hi);
    impl_->names.push_back(joint->name);
    pending = pinocchio::SE3::Identity();
  }
  if (lower.empty())
    throw std::invalid_argument("IK chain has no active joints");
  impl_->lo = Eigen::Map<Q>(lower.data(), lower.size());
  impl_->hi = Eigen::Map<Q>(upper.data(), upper.size());
  impl_->tail = pending;
  impl_->last_q =
      Q::Constant(lower.size(), std::numeric_limits<double>::quiet_NaN());
  impl_->data.reset(new pinocchio::Data(impl_->model));
}

Kinematics::~Kinematics() = default;
const std::vector<std::string>& Kinematics::names() const {
  return impl_->names;
}
const Q& Kinematics::lower() const { return impl_->lo; }
const Q& Kinematics::upper() const { return impl_->hi; }
Mat4 Kinematics::fk(const Q& q) {
  impl_->update(q);
  return impl_->tip.toHomogeneousMatrix();
}
J Kinematics::jacobian(const Q& q) {
  impl_->update(q);
  Eigen::Matrix<double, 6, Eigen::Dynamic> geometric(6, q.size());
  geometric.setZero();
  pinocchio::getJointJacobian(impl_->model, *impl_->data, impl_->last,
                              pinocchio::LOCAL, geometric);
  // At its own joint frame the terminal column is exactly the joint motion
  // subspace. Avoid placement/inverse-placement roundoff creating spurious
  // derivatives for a task structurally independent of that joint.
  geometric.col(q.size() - 1) =
      impl_->data->joints[impl_->last].S().matrix().col(0);
  J result(12, q.size());
  for (int i = 0; i < q.size(); ++i) {
    const Eigen::Vector3d joint_omega = geometric.col(i).tail<3>();
    const Eigen::Vector3d omega =
        impl_->tail.rotation().transpose() * joint_omega;
    result.col(i).head<3>() = impl_->data->oMi[impl_->last].rotation() *
                              (geometric.col(i).head<3>() +
                               joint_omega.cross(impl_->tail.translation()));
    Eigen::Matrix3d skew;
    skew << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(),
        omega.x(), 0;
    Eigen::Matrix3d dr = impl_->tip.rotation() * skew;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) result(3 + r * 3 + c, i) = dr(r, c);
  }
  return result;
}
