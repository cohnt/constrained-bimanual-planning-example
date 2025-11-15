#include "parameterization.h"

std::unique_ptr<drake::planning::IrisParameterizationFunction>
MakeParameterization(bool shoulder_up, bool elbow_up, bool wrist_up) {

    // Capture the booleans by value in lambdas.
    auto parameterization_double =
        [=](const Eigen::VectorXd& x) {
            return IiwaBimanualParameterization<double>(
                x, shoulder_up, elbow_up, wrist_up,
                nullptr);
        };

    auto parameterization_autodiff =
        [=](const Eigen::VectorX<drake::AutoDiffXd>& x) {
            return IiwaBimanualParameterization<drake::AutoDiffXd>(
                x, shoulder_up, elbow_up, wrist_up,
                nullptr);
        };

    bool is_threadsafe = true;

    return std::make_unique<drake::planning::IrisParameterizationFunction>(
        parameterization_double, parameterization_autodiff,
        is_threadsafe, 8);
}