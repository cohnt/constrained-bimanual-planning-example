#include "costs.h"

IiwaBimanualPathCost::IiwaBimanualPathCost(int num_positions,
                                           int num_control_points,
                                           bool shoulder_up, bool elbow_up,
                                           bool wrist_up, double grasp_distance,
                                           bool square)
    : drake::solvers::Cost(num_positions * num_control_points), // scalar cost
      num_positions_(num_positions), num_control_points_(num_control_points),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance), square_(square) {
  set_is_thread_safe(true);
}

template <typename T>
T IiwaBimanualPathCost::DoEvalGeneric(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>
        &control_points_flat) const {

  // Row-major to match numpy convention.
  Eigen::Map<
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      control_points(control_points_flat.data(), num_positions_,
                     num_control_points_);

  T total = T(0);
  for (int i = 1; i < num_control_points_; ++i) {
    Eigen::Matrix<T, Eigen::Dynamic, 1> q0 = IiwaBimanualParameterization<T>(
        control_points.col(i - 1), shoulder_up_, elbow_up_, wrist_up_, nullptr,
        grasp_distance_);
    Eigen::Matrix<T, Eigen::Dynamic, 1> q1 = IiwaBimanualParameterization<T>(
        control_points.col(i), shoulder_up_, elbow_up_, wrist_up_, nullptr,
        grasp_distance_);
    Eigen::Matrix<T, Eigen::Dynamic, 1> delta = q1 - q0;
    if (square_) {
      total += delta.squaredNorm();
    } else {
      total += delta.norm();
    }
  }
  return total;
}

void IiwaBimanualPathCost::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::VectorXd *y) const {
  (*y)(0) = DoEvalGeneric<double>(x);
}

void IiwaBimanualPathCost::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &x,
    drake::AutoDiffVecXd *y) const {
  (*y)(0) = DoEvalGeneric<drake::AutoDiffXd>(x);
}

void IiwaBimanualPathCost::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &x,
    drake::VectorX<drake::symbolic::Expression> *y) const {
  (*y)(0) = DoEvalGeneric<drake::symbolic::Expression>(x);
}