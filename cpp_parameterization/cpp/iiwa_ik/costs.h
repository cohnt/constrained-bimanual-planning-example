#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/context.h"
#include "iiwa_analytic_ik.h"
#include <Eigen/Dense>
#include <memory>

/**
 * Lifts a parameterized configuration to the full configuration space and
 * returns the parameterization's Jacobian there, using only num_positions
 * partial derivatives.
 *
 * Both costs below use this to keep their gradients linear in the number of
 * control points. Each segment of a path cost depends on just two control
 * points, so differentiating a segment against the whole flattened control
 * point matrix -- which is what propagating AutoDiff straight through the cost
 * does -- costs num_positions * num_control_points partial derivatives per
 * segment, and hence O(num_control_points^2) work overall. Lifting each control
 * point once with num_positions partials, and assembling the gradient from
 * those Jacobians instead, is O(num_control_points). Adjacent segments also
 * share a control point, so each lift serves two segments.
 */
void LiftParameterizationWithJacobian(const Eigen::VectorXd &q_tilde,
                                      bool shoulder_up, bool elbow_up,
                                      bool wrist_up, double grasp_distance,
                                      Eigen::VectorXd *q_full,
                                      Eigen::MatrixXd *jacobian);

/**
 * Path energy cost for a sequence of control points for the subordinate arm.
 * Accumulates squared differences between consecutive configurations.
 * If square is true, returns path energy, otherwise returns path length.
 *
 * The optional scale multiplies the accumulated total. It does not change the
 * minimizer, but it lets the caller normalize costs of different physical
 * units onto a common numerical scale (e.g. so that a solver's relative
 * tolerances mean the same thing for each of several candidate objectives).
 */
class IiwaBimanualPathCost final : public drake::solvers::Cost {
public:
  IiwaBimanualPathCost(int num_positions, int num_control_points,
                       bool shoulder_up, bool elbow_up, bool wrist_up,
                       double grasp_distance, bool square, double scale = 1.0);

private:
  // Generic evaluation template
  template <typename T>
  T DoEvalGeneric(const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>
                      &control_points_flat) const;

  // Overrides for Drake Cost
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd *y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
              drake::AutoDiffVecXd *y) const override;
  void
  DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &x,
         drake::VectorX<drake::symbolic::Expression> *y) const override;

  int num_positions_;
  int num_control_points_;
  bool shoulder_up_, elbow_up_, wrist_up_;
  double grasp_distance_;
  bool square_;
  double scale_;
};

/**
 * Kinetic energy path cost for a sequence of control points, following the
 * Riemannian formulation of Kyaw & Kelly, "Geometry-Aware Sampling-Based
 * Motion Planning on Riemannian Manifolds" (arXiv:2602.00992). The metric
 * tensor is the manipulator's mass matrix M(q), so that the path energy
 *
 *     E = 1/2 \int \dot{q}^T M(q) \dot{q} dt
 *
 * measures the kinetic energy required to traverse the path in a given amount
 * of time. Discretely, over consecutive control points,
 *
 *     E = sum_i (f(q~_i) - f(q~_{i-1}))^T M(f(q~_mid,i)) (f(q~_i) - f(q~_{i-1}))
 *
 * where f is the analytic-IK parameterization (8-D -> 14-D) and
 * q~_mid,i = (q~_i + q~_{i-1}) / 2. Evaluating the metric at the midpoint
 * rather than at an endpoint is the paper's midpoint approximation, which is
 * third-order accurate in the separation between configurations (their
 * Theorem 1). The midpoint is taken in the parameterized space and then
 * lifted, so that the configuration at which M is evaluated lies exactly on
 * the constraint manifold (their retraction midpoint, eq. 8, with the
 * parameterization playing the role of the chart).
 *
 * Differences are taken in the full 14-D space rather than pulling the metric
 * back to the parameterized space as J^T M J. The two agree to leading order,
 * since f(q~_i) - f(q~_{i-1}) ~= J (q~_i - q~_{i-1}), but the former is more
 * accurate for finite steps and needs no Jacobian.
 *
 * As with IiwaBimanualPathCost, scale multiplies the accumulated total.
 */
class IiwaBimanualKineticEnergyPathCost final : public drake::solvers::Cost {
public:
  /** The plant is used only to evaluate the mass matrix; it must outlive this
   *  object, and must have exactly 14 positions (the two arms). An AutoDiffXd
   *  copy of the plant, and contexts for both scalar types, are created and
   *  owned internally. */
  IiwaBimanualKineticEnergyPathCost(
      int num_positions, int num_control_points, bool shoulder_up,
      bool elbow_up, bool wrist_up, double grasp_distance,
      const drake::multibody::MultibodyPlant<double> &plant,
      double scale = 1.0);

private:
  /** Cost of the single segment connecting two parameterized configurations,
   *  excluding the overall scale factor. */
  template <typename T>
  T SegmentCost(const Eigen::Ref<const Eigen::VectorX<T>> &q_tilde_0,
                const Eigen::Ref<const Eigen::VectorX<T>> &q_tilde_1) const;

  /** The same, given the full-space displacement and the lifted midpoint,
   *  which the callers compute once and share between adjacent segments. */
  template <typename T>
  T SegmentCostFromLifted(const Eigen::VectorX<T> &delta,
                          const Eigen::VectorX<T> &q_full_mid) const;



  // Overrides for Drake Cost
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd *y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
              drake::AutoDiffVecXd *y) const override;
  void
  DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &x,
         drake::VectorX<drake::symbolic::Expression> *y) const override {
    throw std::logic_error(
        "IiwaBimanualKineticEnergyPathCost::DoEval() does not work for "
        "symbolic variables.");
  }

  int num_positions_;
  int num_control_points_;
  bool shoulder_up_, elbow_up_, wrist_up_;
  double grasp_distance_;
  double scale_;

  const drake::multibody::MultibodyPlant<double> *plant_{};
  std::unique_ptr<drake::multibody::MultibodyPlant<drake::AutoDiffXd>>
      plant_autodiff_{};
  // Mutable because DoEval is const, but evaluating the mass matrix requires
  // writing the configuration into a context.
  mutable std::unique_ptr<drake::systems::Context<double>> context_{};
  mutable std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>>
      context_autodiff_{};
};
