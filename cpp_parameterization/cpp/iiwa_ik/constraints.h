#pragma once

#include <Eigen/Dense>
#include "drake/solvers/mathematical_program.h"
#include "iiwa_analytic_ik.h"

/** Reachability constraint for the bimanual IIWA arm. 
 *  Maps q_and_psi (8-dimensional) to unclipped values (4-dimensional). */
class IiwaBimanualReachableConstraint final : public drake::solvers::Constraint {
 public:
  IiwaBimanualReachableConstraint(bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance);

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const Eigen::VectorX<T>>& q,
                     Eigen::VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

  bool shoulder_up_{}, elbow_up_{}, wrist_up_{};
  double grasp_distance_{};
};

/** Joint limit constraint for the subordinate arm of the bimanual IIWA. */
class IiwaBimanualJointLimitConstraint final : public drake::solvers::Constraint {
 public:
  IiwaBimanualJointLimitConstraint(const Eigen::VectorXd& lower_bound,
                                   const Eigen::VectorXd& upper_bound,
                                   bool shoulder_up, bool elbow_up, bool wrist_up, double grasp_distance);

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const Eigen::VectorX<T>>& q,
                     Eigen::VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

  bool shoulder_up_{}, elbow_up_{}, wrist_up_{};
  double grasp_distance_{};
};