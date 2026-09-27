#include <memory>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/unique_ptr.h>

#include "constraints.h"
#include "costs.h"
#include "parameterization.h"

namespace py = nanobind;

NB_MODULE(_iiwa_ik, m) {
  py::class_<IiwaBimanualReachableConstraint, drake::solvers::Constraint>(
      m, "IiwaBimanualReachableConstraint")
      .def(py::init<bool, bool, bool, double>(), py::arg("shoulder_up"),
           py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"));
  py::class_<IiwaBimanualJointLimitConstraint, drake::solvers::Constraint>(
      m, "IiwaBimanualJointLimitConstraint")
      .def(py::init<Eigen::VectorXd, Eigen::VectorXd, bool, bool, bool,
                    double>(),
           py::arg("lower_bound"), py::arg("upper_bound"),
           py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"),
           py::arg("grasp_distance"));
  py::class_<IiwaBimanualCollisionFreeConstraint, drake::solvers::Constraint>(
      m, "IiwaBimanualCollisionFreeConstraint")
      .def(py::init<
               bool, bool, bool, double,
               std::shared_ptr<
                   drake::multibody::MinimumDistanceLowerBoundConstraint>>(),
           py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"),
           py::arg("grasp_distance"),
           py::arg("minimum_distance_lower_bound_constraint"));
  py::class_<FullFeasibilityConstraint, drake::solvers::Constraint>(
      m, "FullFeasibilityConstraint")
      .def(py::init<
               Eigen::VectorXd, Eigen::VectorXd, bool, bool, bool, double,
               std::shared_ptr<
                   drake::multibody::MinimumDistanceLowerBoundConstraint>>(),
           py::arg("lower_bound"), py::arg("upper_bound"),
           py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"),
           py::arg("grasp_distance"),
           py::arg("minimum_distance_lower_bound_constraint"));
  py::class_<IiwaBimanualPathCost, drake::solvers::Cost>(m,
                                                         "IiwaBimanualPathCost")
      .def(py::init<int, int, bool, bool, bool, double, bool>(),
           py::arg("num_positions"), py::arg("num_control_points"),
           py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"),
           py::arg("grasp_distance"), py::arg("square"));
  m.def("MakeParameterization", &MakeParameterization, py::arg("shoulder_up"),
        py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"));
}
