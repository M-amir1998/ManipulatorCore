// Python bindings for interactive use of ManipulatorCore (kinematics, IK, dynamics, URDF). The
// rendering pipeline does not need them: it reads the JSON exported by mc_demos.

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/model/urdf.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(manipulatorcore, m) {
  m.doc() = "ManipulatorCore: classical robot manipulation (kinematics, IK, dynamics).";

  py::class_<mc::RobotModel>(m, "RobotModel")
      .def_property_readonly("name", &mc::RobotModel::name)
      .def_property_readonly("dof", &mc::RobotModel::dof)
      .def_property_readonly("lower_limits", &mc::RobotModel::lowerLimits)
      .def_property_readonly("upper_limits", &mc::RobotModel::upperLimits)
      .def("__repr__", [](const mc::RobotModel& r) {
        return "<RobotModel '" + r.name() + "' dof=" + std::to_string(r.dof()) + ">";
      });

  m.def("make_planar_2r", [] { return mc::makePlanar2R(); });
  m.def("make_planar_3r", [] { return mc::makePlanar3R(); });
  m.def("make_industrial_6r", &mc::makeIndustrial6R);
  m.def("industrial_6r_home", &mc::industrial6RHome);
  m.def("make_redundant_7r", &mc::makeRedundant7R);
  m.def("redundant_7r_home", &mc::redundant7RHome);
  m.def("load_urdf", [](const std::string& xml) { return mc::fromURDF(xml); }, py::arg("urdf_xml"));
  m.def("to_urdf", [](const mc::RobotModel& r) { return mc::toURDF(r); });

  m.def(
      "end_effector_pose",
      [](const mc::RobotModel& r, const mc::VecX& q) { return mc::endEffectorPose(r, q).matrix(); },
      "4x4 pose of the tool frame in the base frame (T_base_ee).");
  m.def(
      "geometric_jacobian",
      [](const mc::RobotModel& r, const mc::VecX& q) { return mc::MatX(mc::geometricJacobian(r, q)); },
      "6 x n geometric Jacobian, rows [omega; v].");
  m.def("space_jacobian", [](const mc::RobotModel& r, const mc::VecX& q) { return mc::MatX(mc::spaceJacobian(r, q)); });
  m.def("body_jacobian", [](const mc::RobotModel& r, const mc::VecX& q) { return mc::MatX(mc::bodyJacobian(r, q)); });
  m.def("manipulability",
        [](const mc::RobotModel& r, const mc::VecX& q) { return mc::analyzeSingularity(r, q).full.manipulability; });

  py::class_<mc::IKResult>(m, "IKResult")
      .def_readonly("q", &mc::IKResult::q)
      .def_readonly("iterations", &mc::IKResult::iterations)
      .def_readonly("position_error", &mc::IKResult::position_error)
      .def_readonly("orientation_error", &mc::IKResult::orientation_error)
      .def_property_readonly("converged", &mc::IKResult::converged)
      .def_property_readonly("status", [](const mc::IKResult& r) { return std::string(mc::toString(r.status)); });

  m.def(
      "solve_ik",
      [](const mc::RobotModel& r, const mc::Mat4& target, const mc::VecX& seed, const std::string& method,
         int max_iterations) {
        mc::IKOptions o;
        if (method == "transpose") {
          o.method = mc::IKMethod::JacobianTranspose;
        } else if (method == "pinv") {
          o.method = mc::IKMethod::PseudoInverse;
        } else if (method == "lm") {
          o.method = mc::IKMethod::LevenbergMarquardt;
        } else {
          o.method = mc::IKMethod::DampedLeastSquares;
        }
        o.max_iterations = max_iterations;
        return mc::solveIK(r, mc::SE3::fromMatrix(target, 1e-6), seed, o);
      },
      py::arg("robot"), py::arg("target"), py::arg("seed"), py::arg("method") = "dls", py::arg("max_iterations") = 200);

  m.def("mass_matrix", &mc::massMatrix);
  m.def("gravity_torques", &mc::gravityTorques);
  m.def("inverse_dynamics", &mc::inverseDynamics);
  m.def("forward_dynamics", [](const mc::RobotModel& r, const mc::VecX& q, const mc::VecX& qd, const mc::VecX& tau) {
    return mc::forwardDynamics(r, q, qd, tau);
  });

  py::register_exception<mc::InvalidArgument>(m, "InvalidArgument", PyExc_ValueError);
}
