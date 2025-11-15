#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "parameterization.h"
#include "constraints.h"

namespace py = pybind11;

PYBIND11_MODULE(_iiwa_ik, m) {
    py::class_<IiwaBimanualReachableConstraint, drake::solvers::Constraint, std::shared_ptr<IiwaBimanualReachableConstraint>>(
        m, "IiwaBimanualReachableConstraint")
        .def(py::init<bool, bool, bool>(), 
             py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"));
    py::class_<IiwaBimanualJointLimitConstraint, drake::solvers::Constraint, std::shared_ptr<IiwaBimanualJointLimitConstraint>>(
        m, "IiwaBimanualJointLimitConstraint")
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, bool, bool, bool>(),
             py::arg("lower_bound"), py::arg("upper_bound"),
             py::arg("shoulder_up"), py::arg("elbow_up"), py::arg("wrist_up"));
    m.def("MakeParameterization", &MakeParameterization, "Create a Drake IrisParameterizationFunction");
}