// Project RoboOrchard
// Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Geometry>
#include <mutex>
#include <stdexcept>

#include "ik.hpp"

namespace py = pybind11;
namespace {
int orientation(const std::optional<std::string>& mode) {
  if (!mode) return -1;
  if (*mode == "X") return 0;
  if (*mode == "Y") return 1;
  if (*mode == "Z") return 2;
  if (*mode == "all") return 3;
  throw std::invalid_argument("Unknown orientation mode: " + *mode);
}
py::dict pack(const Result& r) {
  py::dict result;
  result["q"] = std::vector<double>(r.q.data(), r.q.data() + r.q.size());
  result["accepted"] = r.accepted;
  result["nfev"] = r.nfev;
  result["njev"] = r.njev;
  result["status"] = r.status;
  result["cost"] = r.cost;
  result["optimality"] = r.optimality;
  result["position_error"] = r.position_error;
  result["rotation_error"] = r.rotation_error;
  return result;
}
// Pinocchio Data and the same-q cache belong to one solver instance.
// Hold the GIL for diagnostics and release it only under the per-instance lock.
struct Ik {
  Solver solver;
  std::mutex mutex;
  Ik(const std::string& path, const std::string& base,
     const std::string& ee)
      : solver(path, base, ee) {}
};
}  // namespace

PYBIND11_MODULE(_ik, m) {
  m.doc() = "Pinocchio chain kinematics and bounded TRF IK";
  py::class_<Ik>(m, "Ik")
      .def(py::init<const std::string&, const std::string&,
                    const std::string&>(),
           py::arg("urdf_path"), py::arg("base_link"), py::arg("ee_link"))
      .def_property_readonly("joint_names",
                             [](Ik& s) { return s.solver.names(); })
      .def_property_readonly(
          "num_joints", [](Ik& s) { return s.solver.names().size(); })
      .def_property_readonly("lower",
                             [](Ik& s) { return s.solver.lower(); })
      .def_property_readonly("upper",
                             [](Ik& s) { return s.solver.upper(); })
      .def("clamp", [](Ik& s, const Q& q) { return s.solver.clamp(q); })
      .def("fk",
           [](Ik& s, const Q& q) {
             std::lock_guard<std::mutex> lock(s.mutex);
             return s.solver.fk(q);
           })
      .def("task_jacobian",
           [](Ik& s, const Q& q) {
             std::lock_guard<std::mutex> lock(s.mutex);
             return s.solver.task_jacobian(q);
           })
      .def(
          "solve_pose",
          [](Ik& s, const std::vector<double>& pose,
             const std::optional<Q>& seed,
             const std::optional<std::string>& mode,
             const std::optional<double>& regularization, bool clip,
             double ptol, double rtol) {
            if (pose.size() != 7)
              throw std::invalid_argument(
                  "Expected xyz xyzw pose of length seven");
            Eigen::Quaterniond quat(pose[6], pose[3], pose[4], pose[5]);
            if (!quat.coeffs().allFinite() || quat.norm() == 0)
              throw std::invalid_argument(
                  "Found zero norm or non-finite quaternion");
            quat.normalize();
            Mat4 target = Mat4::Identity();
            target.topLeftCorner<3, 3>() = quat.toRotationMatrix();
            target.topRightCorner<3, 1>() =
                Eigen::Vector3d(pose[0], pose[1], pose[2]);
            Options options;
            options.orientation = orientation(mode);
            options.regularization = regularization.value_or(0);
            options.clip = clip;
            options.position_tolerance = ptol;
            options.orientation_tolerance = rtol;
            Result result;
            {
              py::gil_scoped_release release;
              std::lock_guard<std::mutex> lock(s.mutex);
              result = s.solver.solve(target, seed, options);
            }
            return pack(result);
          },
          py::arg("pose"), py::arg("seed") = py::none(),
          py::arg("orientation_mode") = "all",
          py::arg("regularization_parameter") = .02,
          py::arg("clip_to_limit") = true, py::arg("position_tolerance") = .05,
          py::arg("orientation_tolerance") = .3);
}
