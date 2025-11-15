#pragma once

#include <memory>
#include <Eigen/Dense>
#include "drake/planning/iris/iris_common.h"
#include "iiwa_analytic_ik.h"

/** Factory function that returns a parameterization function object
 *  for the bimanual IIWA, with the given shoulder/elbow/wrist configuration. */
std::unique_ptr<drake::planning::IrisParameterizationFunction> MakeParameterization(
    const bool shoulder_up, const bool elbow_up, const bool wrist_up, const double grasp_distance);