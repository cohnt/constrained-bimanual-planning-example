#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "parameterization.h"
#include "constraints.h"

namespace py = pybind11;

PYBIND11_MODULE(_iiwa_ik, m) {
    py::class_<IiwaBimanualReachableConstraint, drake::solvers::Constraint, std::shared_ptr<IiwaBimanualReachableConstraint>>(
        m, "IiwaBimanualReachableConstraint")
        .def(py::init<bool, bool, bool, double>(), 
             py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"));
    py::class_<IiwaBimanualJointLimitConstraint, drake::solvers::Constraint, std::shared_ptr<IiwaBimanualJointLimitConstraint>>(
        m, "IiwaBimanualJointLimitConstraint")
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, bool, bool, bool, double>(),
             py::arg("lower_bound"), py::arg("upper_bound"),
             py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"));
    py::class_<IiwaBimanualCollisionFreeConstraint, drake::solvers::Constraint, std::shared_ptr<IiwaBimanualCollisionFreeConstraint>>(
        m, "IiwaBimanualCollisionFreeConstraint")
        .def(py::init<bool, bool, bool, double, std::shared_ptr<drake::multibody::MinimumDistanceLowerBoundConstraint>>(),
            py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"),
             py::arg("minimum_distance_lower_bound_constraint"));
    py::class_<FullFeasibilityConstraint, drake::solvers::Constraint, std::shared_ptr<FullFeasibilityConstraint>>(
        m, "FullFeasibilityConstraint")
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, bool, bool, bool, double, std::shared_ptr<drake::multibody::MinimumDistanceLowerBoundConstraint>>(),
             py::arg("lower_bound"), py::arg("upper_bound"),
            py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"),
             py::arg("minimum_distance_lower_bound_constraint"));
    m.def("MakeParameterization", &MakeParameterization, py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"), py::arg("grasp_distance"));
}