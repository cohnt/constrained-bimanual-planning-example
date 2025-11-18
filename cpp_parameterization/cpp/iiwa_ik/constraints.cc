#include "constraints.h"
#include <cassert>

IiwaBimanualReachableConstraint::IiwaBimanualReachableConstraint(
    bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance)
    : drake::solvers::Constraint(4, // output dimension
                                 8, // input dimension
                                 Eigen::Vector4d::Constant(-(1.0 - 1e-4)),
                                 Eigen::Vector4d::Constant(1.0 - 1e-4)),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance) {
  set_is_thread_safe(true);
}

template <typename T>
void IiwaBimanualReachableConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>> &q, Eigen::VectorX<T> *y) const {
  Eigen::VectorX<T> unclipped;
  IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_,
                                  &unclipped, grasp_distance_);
  *y = unclipped; // length 4
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::VectorXd *y) const {
  DoEvalGeneric<double>(q, y);
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &q,
    drake::AutoDiffVecXd *y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}

void IiwaBimanualReachableConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &q,
    drake::VectorX<drake::symbolic::Expression> *y) const {
  DoEvalGeneric<drake::symbolic::Expression>(q, y);
}

// --------------------------------------------------

IiwaBimanualJointLimitConstraint::IiwaBimanualJointLimitConstraint(
    const Eigen::VectorXd &lower_bound, const Eigen::VectorXd &upper_bound,
    bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance)
    : drake::solvers::Constraint(lower_bound.size(), 8, lower_bound,
                                 upper_bound),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance) {
  set_is_thread_safe(true);
}

template <typename T>
void IiwaBimanualJointLimitConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>> &q, Eigen::VectorX<T> *y) const {
  // Only the subordinate arm's joints matter.
  *y = IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_,
                                       nullptr, grasp_distance_)
           .tail(7);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::VectorXd *y) const {
  DoEvalGeneric<double>(q, y);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &q,
    drake::AutoDiffVecXd *y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}

void IiwaBimanualJointLimitConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &q,
    drake::VectorX<drake::symbolic::Expression> *y) const {
  DoEvalGeneric<drake::symbolic::Expression>(q, y);
}

// --------------------------------------------------

IiwaBimanualCollisionFreeConstraint::IiwaBimanualCollisionFreeConstraint(
    bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance,
    std::shared_ptr<drake::multibody::MinimumDistanceLowerBoundConstraint>
        minimum_distance_lower_bound_constraint)
    : drake::solvers::Constraint(
          1, 8,
          Eigen::VectorXd::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          Eigen::VectorXd::Constant(1, 1.0)),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance),
      minimum_distance_lower_bound_constraint_(
          minimum_distance_lower_bound_constraint) {
  assert(minimum_distance_lower_bound_constraint_ != nullptr);
  set_is_thread_safe(true);
}

template <typename T>
void IiwaBimanualCollisionFreeConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>> &q, Eigen::VectorX<T> *y) const {
  Eigen::VectorX<T> q_full = IiwaBimanualParameterization<T>(
      q, shoulder_up_, elbow_up_, wrist_up_, nullptr, grasp_distance_);
  minimum_distance_lower_bound_constraint_->Eval(q_full, y);
}

void IiwaBimanualCollisionFreeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::VectorXd *y) const {
  DoEvalGeneric<double>(q, y);
}

void IiwaBimanualCollisionFreeConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &q,
    drake::AutoDiffVecXd *y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}

// --------------------------------------------------

// The output vector is stacked in the order (unclipped values, subordinate arm
// limits, min distance constraint)
FullFeasibilityConstraint::FullFeasibilityConstraint(
    const Eigen::VectorXd &joint_lower, const Eigen::VectorXd &joint_upper,
    bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance,
    std::shared_ptr<drake::multibody::MinimumDistanceLowerBoundConstraint>
        md_constraint)
    : drake::solvers::Constraint(
          /*num_outputs=*/4 + joint_lower.size() + 1,
          /*num_inputs=*/8,
          [&]() {
            Eigen::VectorXd lb(4 + joint_lower.size() + 1);
            lb.head(4).setConstant(-(1.0 - 1e-4)); // reachability
            lb.segment(4, joint_lower.size()) = joint_lower;
            lb(4 + joint_lower.size()) =
                -std::numeric_limits<double>::infinity();
            return lb;
          }(),
          [&]() {
            Eigen::VectorXd ub(4 + joint_lower.size() + 1);
            ub.head(4).setConstant(1.0 - 1e-4);
            ub.segment(4, joint_lower.size()) = joint_upper;
            ub(4 + joint_lower.size()) = 1.0;
            return ub;
          }()),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance),
      minimum_distance_lower_bound_constraint_(std::move(md_constraint)) {
  assert(minimum_distance_lower_bound_constraint_ != nullptr);
  set_is_thread_safe(true);
}

template <typename T>
void FullFeasibilityConstraint::DoEvalGeneric(
    const Eigen::Ref<const Eigen::VectorX<T>> &q, Eigen::VectorX<T> *y) const {
  y->resize(12);

  // Parameterization: unclipped reachability output
  Eigen::VectorX<T> unclipped(4);
  Eigen::VectorX<T> q_full = IiwaBimanualParameterization<T>(
      q, shoulder_up_, elbow_up_, wrist_up_, &unclipped, grasp_distance_);

  // Evaluate collision distance
  Eigen::VectorX<T> min_distance_output(1);
  minimum_distance_lower_bound_constraint_->Eval(q_full, &min_distance_output);

  // Assign output segments
  y->segment(0, 4) = unclipped;            // reachability
  y->segment(4, 7) = q_full.tail(7);       // joint limits
  y->segment(11, 1) = min_distance_output; // collision
}

void FullFeasibilityConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::VectorXd *y) const {
  DoEvalGeneric<double>(q, y);
}

void FullFeasibilityConstraint::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &q,
    drake::AutoDiffVecXd *y) const {
  DoEvalGeneric<drake::AutoDiffXd>(q, y);
}