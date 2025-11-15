#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "constraints.cc"
#include "parameterization.cc"

namespace py = pybind11;

PYBIND11_MODULE(iiwa_ik, m) {

    // TODO REMOVE
    m.def("safe_arccos",
          py::overload_cast<const double&, double, double>(&SafeArccos<double>),
          py::arg("val"), py::arg("a"), py::arg("b"));
    m.def("safe_arccos",
          py::overload_cast<const drake::AutoDiffXd&, double, double>(&SafeArccos<drake::AutoDiffXd>),
          py::arg("val"), py::arg("a"), py::arg("b"));

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