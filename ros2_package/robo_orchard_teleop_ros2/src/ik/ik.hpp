// Project RoboOrchard
// Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <Eigen/Core>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using Q = Eigen::VectorXd;
using F = Eigen::VectorXd;
using J = Eigen::MatrixXd;
using Mat4 = Eigen::Matrix4d;

class Kinematics {
 public:
  Kinematics(const std::string& urdf_path, const std::string& base,
             const std::string& ee);
  ~Kinematics();
  Mat4 fk(const Q& q);
  J jacobian(const Q& q);
  const std::vector<std::string>& names() const;
  const Q& lower() const;
  const Q& upper() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Options {
  // -1: position only; 0/1/2: X/Y/Z; 3: all matrix entries.
  int orientation = 3;
  double regularization = 0;
  bool clip = true;
  double position_tolerance = .05, orientation_tolerance = .3;
  double ftol = 1e-8, xtol = 1e-8, gtol = 1e-8;
  int max_nfev = 0;  // SciPy's default: 100 * number of active joints.
};

struct Result {
  Q q;
  int nfev = 1, njev = 1, status = 0;
  double cost = 0, optimality = 0, position_error = 0, rotation_error = 0;
  bool accepted = false;
};

class Solver {
 public:
  Solver(const std::string& urdf_path, const std::string& base,
         const std::string& ee);
  Mat4 fk(const Q& q) { return kin_.fk(q); }
  J task_jacobian(const Q& q) { return kin_.jacobian(q); }
  const std::vector<std::string>& names() const { return kin_.names(); }
  const Q& lower() const { return lower_; }
  const Q& upper() const { return upper_; }
  Q clamp(const Q& seed) const;
  Result solve(const Mat4& target, const std::optional<Q>& seed,
               const Options& options);

 private:
  Kinematics kin_;
  Q lower_, upper_;
  double difference_step(const Q& q, int i) const;
  F residual(const Q& q, const Mat4& target, const Q& seed,
             const Options& options);
  J residual_jacobian(const Q& q, const Q& seed, const Options& options);
};
