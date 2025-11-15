#include "constraints.h"

IiwaBimanualReachableConstraint::IiwaBimanualReachableConstraint(
    bool shoulder_up, bool elbow_up, bool wrist_up)
    : drake::solvers::Constraint(4,  // output dimension
                                 8,  // input dimension
                                 Eigen::Vector4d::Constant(-1.0),
                                 Eigen::Vector4d::Constant(1.0)),
      shoulder_up_(shoulder_up),
      elbow_up_(elbow_up),
      wrist_up_(wrist_up) {
  set_is_thread_safe(true);
}

template <typename T>
void IiwaBimanualReachableConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>>& q, Eigen::VectorX<T>* y) const {
  Eigen::VectorX<T> unclipped;
  IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_, &unclipped);
  *y = unclipped;  // length 4
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(q, y);
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd>& q,
    drake::AutoDiffVecXd* y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  DoEvalGeneric<drake::symbolic::Expression>(q, y);
}

// --------------------------------------------------

IiwaBimanualJointLimitConstraint::IiwaBimanualJointLimitConstraint(
    const Eigen::VectorXd& lower_bound,
    const Eigen::VectorXd& upper_bound,
    bool shoulder_up, bool elbow_up, bool wrist_up)
    : drake::solvers::Constraint(lower_bound.size(), 8, lower_bound, upper_bound),
      shoulder_up_(shoulder_up),
      elbow_up_(elbow_up),
      wrist_up_(wrist_up) {
  set_is_thread_safe(true);
}

template <typename T>
void IiwaBimanualJointLimitConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>>& q, Eigen::VectorX<T>* y) const {
  // Only the subordinate arm's joints matter.
  *y = IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_, nullptr)
           .tail(7);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(q, y);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd>& q,
    drake::AutoDiffVecXd* y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  DoEvalGeneric<drake::symbolic::Expression>(q, y);
}