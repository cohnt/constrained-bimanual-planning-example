#pragma once

#include <Eigen/Dense>
#include "drake/solvers/mathematical_program.h"
#include "iiwa_analytic_ik.h"

/**
 * Path energy cost for a sequence of control points for the subordinate arm.
 * Accumulates squared differences between consecutive configurations.
 * If square is true, returns path energy, otherwise returns path length.
 */
class IiwaBimanualPathCost final : public drake::solvers::Cost {
 public:
  IiwaBimanualPathCost(int num_positions, int num_control_points,
                       bool shoulder_up, bool elbow_up, bool wrist_up,
                       double grasp_distance, bool square);

 private:
  // Generic evaluation template
  template <typename T>
  T DoEvalGeneric(const Eigen::Ref<const Eigen::Matrix<T,Eigen::Dynamic,1>>& control_points_flat) const;

  // Overrides for Drake Cost
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

  int num_positions_;
  int num_control_points_;
  bool shoulder_up_, elbow_up_, wrist_up_;
  double grasp_distance_;
  bool square_;
};