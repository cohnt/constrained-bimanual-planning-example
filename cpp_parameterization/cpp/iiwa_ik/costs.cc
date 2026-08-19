#include "costs.h"

#include "drake/math/autodiff_gradient.h"

#include <type_traits>
#include <vector>

void LiftParameterizationWithJacobian(const Eigen::VectorXd &q_tilde,
                                      bool shoulder_up, bool elbow_up,
                                      bool wrist_up, double grasp_distance,
                                      Eigen::VectorXd *q_full,
                                      Eigen::MatrixXd *jacobian) {
  const drake::AutoDiffVecXd q_tilde_autodiff =
      drake::math::InitializeAutoDiff(q_tilde);
  const drake::AutoDiffVecXd q_full_autodiff =
      IiwaBimanualParameterization<drake::AutoDiffXd>(
          q_tilde_autodiff, shoulder_up, elbow_up, wrist_up, nullptr,
          grasp_distance);
  *q_full = drake::math::ExtractValue(q_full_autodiff);
  *jacobian = drake::math::ExtractGradient(q_full_autodiff);
}

// Scatters a gradient with respect to one control point's coordinates into the
// gradient with respect to the flattened control point matrix. The matrix is
// row-major to match numpy convention, so element (r, c) lives at flat index
// r * num_control_points + c.
namespace {
void ScatterControlPointGradient(
    const Eigen::Ref<const drake::AutoDiffVecXd> &x,
    const Eigen::VectorXd &d_control_point, int column, int num_control_points,
    Eigen::VectorXd *total_gradient) {
  for (int r = 0; r < d_control_point.size(); ++r) {
    // Note that drake::AutoDiffXd::derivatives() returns an expression whose
    // underlying AutoDiffXd is a temporary here, so it must be evaluated into a
    // vector rather than bound to a reference.
    const Eigen::VectorXd row = x(r * num_control_points + column).derivatives();
    if (row.size() > 0) {
      *total_gradient += d_control_point(r) * row;
    }
  }
}
}  // namespace

IiwaBimanualPathCost::IiwaBimanualPathCost(int num_positions,
                                           int num_control_points,
                                           bool shoulder_up, bool elbow_up,
                                           bool wrist_up, double grasp_distance,
                                           bool square, double scale)
    : drake::solvers::Cost(num_positions * num_control_points), // scalar cost
      num_positions_(num_positions), num_control_points_(num_control_points),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance), square_(square), scale_(scale) {
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
  // Consecutive segments share a control point, so each one is lifted once and
  // carried forward rather than lifted again as the next segment's start.
  Eigen::Matrix<T, Eigen::Dynamic, 1> q0 = IiwaBimanualParameterization<T>(
      control_points.col(0), shoulder_up_, elbow_up_, wrist_up_, nullptr,
      grasp_distance_);
  for (int i = 1; i < num_control_points_; ++i) {
    Eigen::Matrix<T, Eigen::Dynamic, 1> q1 = IiwaBimanualParameterization<T>(
        control_points.col(i), shoulder_up_, elbow_up_, wrist_up_, nullptr,
        grasp_distance_);
    Eigen::Matrix<T, Eigen::Dynamic, 1> delta = q1 - q0;
    if (square_) {
      total += delta.squaredNorm();
    } else {
      total += delta.norm();
    }
    q0 = std::move(q1);
  }
  return scale_ * total;
}

void IiwaBimanualPathCost::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::VectorXd *y) const {
  (*y)(0) = DoEvalGeneric<double>(x);
}

void IiwaBimanualPathCost::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &x,
    drake::AutoDiffVecXd *y) const {
  // Rather than propagating AutoDiff through DoEvalGeneric, which would be
  // quadratic in the number of control points, lift each control point once
  // with num_positions_ partials and assemble the gradient from the resulting
  // Jacobians. See LiftParameterizationWithJacobian in costs.h.
  //
  // For a segment with delta = f(q~_1) - f(q~_0), the cost is either
  // delta^T delta or |delta|, so
  //
  //   d(cost)/d(q~_0) = -J_0^T g   and   d(cost)/d(q~_1) = J_1^T g,
  //
  // with g = 2 delta for the energy form and g = delta / |delta| for the
  // length form.
  const int n = num_positions_;

  // Row-major to match numpy convention.
  Eigen::Map<const Eigen::Matrix<drake::AutoDiffXd, Eigen::Dynamic,
                                 Eigen::Dynamic, Eigen::RowMajor>>
      control_points(x.data(), n, num_control_points_);

  int num_derivatives = 0;
  for (int i = 0; i < x.size(); ++i) {
    if (x(i).derivatives().size() > 0) {
      num_derivatives = x(i).derivatives().size();
      break;
    }
  }

  double total_value = 0.0;
  Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(num_derivatives);

  Eigen::VectorXd q_tilde_0(n), q_tilde_1(n);
  for (int r = 0; r < n; ++r) q_tilde_0(r) = control_points(r, 0).value();

  Eigen::VectorXd q_full_0, q_full_1;
  Eigen::MatrixXd jacobian_0, jacobian_1;
  LiftParameterizationWithJacobian(q_tilde_0, shoulder_up_, elbow_up_,
                                   wrist_up_, grasp_distance_, &q_full_0,
                                   &jacobian_0);

  for (int i = 1; i < num_control_points_; ++i) {
    for (int r = 0; r < n; ++r) q_tilde_1(r) = control_points(r, i).value();
    LiftParameterizationWithJacobian(q_tilde_1, shoulder_up_, elbow_up_,
                                     wrist_up_, grasp_distance_, &q_full_1,
                                     &jacobian_1);
    const Eigen::VectorXd delta = q_full_1 - q_full_0;

    Eigen::VectorXd d_delta;
    if (square_) {
      total_value += delta.squaredNorm();
      d_delta = 2.0 * delta;
    } else {
      const double norm = delta.norm();
      total_value += norm;
      // The length form is not differentiable where two control points
      // coincide; the subgradient there contains zero, so use it.
      d_delta = (norm > 1e-14) ? Eigen::VectorXd(delta / norm)
                               : Eigen::VectorXd::Zero(delta.size());
    }

    ScatterControlPointGradient(x, -(jacobian_0.transpose() * d_delta), i - 1,
                                num_control_points_, &total_gradient);
    ScatterControlPointGradient(x, jacobian_1.transpose() * d_delta, i,
                                num_control_points_, &total_gradient);

    q_tilde_0 = q_tilde_1;
    q_full_0 = q_full_1;
    jacobian_0 = jacobian_1;
  }

  (*y)(0) = drake::AutoDiffXd(scale_ * total_value, scale_ * total_gradient);
}

void IiwaBimanualPathCost::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>> &x,
    drake::VectorX<drake::symbolic::Expression> *y) const {
  (*y)(0) = DoEvalGeneric<drake::symbolic::Expression>(x);
}

// --------------------------------------------------

IiwaBimanualKineticEnergyPathCost::IiwaBimanualKineticEnergyPathCost(
    int num_positions, int num_control_points, bool shoulder_up, bool elbow_up,
    bool wrist_up, double grasp_distance,
    const drake::multibody::MultibodyPlant<double> &plant, double scale)
    : drake::solvers::Cost(num_positions * num_control_points), // scalar cost
      num_positions_(num_positions), num_control_points_(num_control_points),
      shoulder_up_(shoulder_up), elbow_up_(elbow_up), wrist_up_(wrist_up),
      grasp_distance_(grasp_distance), scale_(scale), plant_(&plant) {
  DRAKE_THROW_UNLESS(plant.num_positions() == 14);
  using drake::multibody::MultibodyPlant;
  plant_autodiff_ =
      drake::systems::System<double>::ToAutoDiffXd<MultibodyPlant>(plant);
  context_ = plant_->CreateDefaultContext();
  context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  // The contexts are mutable state shared by every evaluation, so this cost
  // must not be evaluated concurrently.
  set_is_thread_safe(false);
}

template <typename T>
T IiwaBimanualKineticEnergyPathCost::SegmentCost(
    const Eigen::Ref<const Eigen::VectorX<T>> &q_tilde_0,
    const Eigen::Ref<const Eigen::VectorX<T>> &q_tilde_1) const {
  const Eigen::VectorX<T> q_full_0 = IiwaBimanualParameterization<T>(
      q_tilde_0, shoulder_up_, elbow_up_, wrist_up_, nullptr, grasp_distance_);
  const Eigen::VectorX<T> q_full_1 = IiwaBimanualParameterization<T>(
      q_tilde_1, shoulder_up_, elbow_up_, wrist_up_, nullptr, grasp_distance_);
  const Eigen::VectorX<T> q_tilde_mid = 0.5 * (q_tilde_0 + q_tilde_1);
  return SegmentCostFromLifted<T>(
      q_full_1 - q_full_0,
      IiwaBimanualParameterization<T>(q_tilde_mid, shoulder_up_, elbow_up_,
                                      wrist_up_, nullptr, grasp_distance_));
}

template <typename T>
T IiwaBimanualKineticEnergyPathCost::SegmentCostFromLifted(
    const Eigen::VectorX<T> &delta, const Eigen::VectorX<T> &q_full_mid) const {
  Eigen::MatrixX<T> mass_matrix(14, 14);
  if constexpr (std::is_same_v<T, double>) {
    plant_->SetPositions(context_.get(), q_full_mid);
    plant_->CalcMassMatrix(*context_, &mass_matrix);
  } else {
    plant_autodiff_->SetPositions(context_autodiff_.get(), q_full_mid);
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &mass_matrix);
  }
  return delta.dot(mass_matrix * delta);
}

void IiwaBimanualKineticEnergyPathCost::DoEval(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd *y) const {
  // Row-major to match numpy convention.
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
      control_points(x.data(), num_positions_, num_control_points_);

  double total = 0.0;
  // As above, each control point is lifted once and carried forward.
  Eigen::VectorXd q_full_0 = IiwaBimanualParameterization<double>(
      Eigen::VectorXd(control_points.col(0)), shoulder_up_, elbow_up_,
      wrist_up_, nullptr, grasp_distance_);
  for (int i = 1; i < num_control_points_; ++i) {
    const Eigen::VectorXd q_tilde_0 = control_points.col(i - 1);
    const Eigen::VectorXd q_tilde_1 = control_points.col(i);
    Eigen::VectorXd q_full_1 = IiwaBimanualParameterization<double>(
        q_tilde_1, shoulder_up_, elbow_up_, wrist_up_, nullptr,
        grasp_distance_);
    const Eigen::VectorXd q_full_mid = IiwaBimanualParameterization<double>(
        Eigen::VectorXd(0.5 * (q_tilde_0 + q_tilde_1)), shoulder_up_,
        elbow_up_, wrist_up_, nullptr, grasp_distance_);
    total += SegmentCostFromLifted<double>(q_full_1 - q_full_0, q_full_mid);
    q_full_0 = std::move(q_full_1);
  }
  (*y)(0) = scale_ * total;
}

void IiwaBimanualKineticEnergyPathCost::DoEval(
    const Eigen::Ref<const drake::AutoDiffVecXd> &x,
    drake::AutoDiffVecXd *y) const {
  const int n = num_positions_;

  // The cost of one segment is delta^T M(f(q~_mid)) delta, where
  // delta = f(q~_1) - f(q~_0) and q~_mid = (q~_0 + q~_1) / 2. Its derivative
  // with respect to a decision variable v splits into two pieces,
  //
  //   d/dv [delta^T M delta] = 2 (M delta)^T d(delta)/dv
  //                            + delta^T [dM/dq~_mid] [dq~_mid/dv] delta.
  //
  // The second piece is the expensive one, and the mass matrix depends on the
  // segment's 2 * n decision variables only through the n-dimensional midpoint,
  // so it needs only n partial derivatives rather than 2 * n. The chain back
  // out is trivial, since d(q~_mid)/d(q~_0) = d(q~_mid)/d(q~_1) = I / 2.
  //
  // The first piece needs the parameterization's Jacobian at each control
  // point, which is n partials as well, and which each of the two segments
  // touching that control point can share.

  // Row-major to match numpy convention; element (r, c) of the control point
  // matrix lives at flat index r * num_control_points_ + c.
  Eigen::Map<const Eigen::Matrix<drake::AutoDiffXd, Eigen::Dynamic,
                                 Eigen::Dynamic, Eigen::RowMajor>>
      control_points(x.data(), n, num_control_points_);

  int num_derivatives = 0;
  for (int i = 0; i < x.size(); ++i) {
    if (x(i).derivatives().size() > 0) {
      num_derivatives = x(i).derivatives().size();
      break;
    }
  }

  double total_value = 0.0;
  Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(num_derivatives);

  auto Scatter = [&](const Eigen::VectorXd &d_control_point, int column) {
    ScatterControlPointGradient(x, d_control_point, column,
                                num_control_points_, &total_gradient);
  };

  Eigen::VectorXd q_tilde_0(n), q_tilde_1(n);
  for (int r = 0; r < n; ++r) q_tilde_0(r) = control_points(r, 0).value();

  Eigen::VectorXd q_full_0, q_full_1, q_full_mid;
  Eigen::MatrixXd jacobian_0, jacobian_1;
  LiftParameterizationWithJacobian(q_tilde_0, shoulder_up_, elbow_up_,
                                  wrist_up_, grasp_distance_, &q_full_0,
                                  &jacobian_0);

  for (int i = 1; i < num_control_points_; ++i) {
    for (int r = 0; r < n; ++r) q_tilde_1(r) = control_points(r, i).value();
    LiftParameterizationWithJacobian(q_tilde_1, shoulder_up_, elbow_up_,
                                    wrist_up_, grasp_distance_, &q_full_1,
                                    &jacobian_1);
    const Eigen::VectorXd delta = q_full_1 - q_full_0;

    // The mass matrix, differentiated with respect to the midpoint only.
    const drake::AutoDiffVecXd q_tilde_mid_autodiff =
        drake::math::InitializeAutoDiff(
            Eigen::VectorXd(0.5 * (q_tilde_0 + q_tilde_1)));
    const drake::AutoDiffVecXd q_full_mid_autodiff =
        IiwaBimanualParameterization<drake::AutoDiffXd>(
            q_tilde_mid_autodiff, shoulder_up_, elbow_up_, wrist_up_, nullptr,
            grasp_distance_);
    plant_autodiff_->SetPositions(context_autodiff_.get(), q_full_mid_autodiff);
    Eigen::MatrixX<drake::AutoDiffXd> mass_matrix_autodiff(14, 14);
    plant_autodiff_->CalcMassMatrix(*context_autodiff_, &mass_matrix_autodiff);

    // Holding delta constant here isolates the dM term; the delta term is
    // added separately below from the Jacobians.
    const drake::AutoDiffVecXd delta_constant =
        delta.cast<drake::AutoDiffXd>();
    const drake::AutoDiffXd segment =
        delta_constant.dot(mass_matrix_autodiff * delta_constant);
    total_value += segment.value();
    // An empty derivatives vector means "constant", which for a configuration
    // the mass matrix genuinely does not depend on would otherwise produce a
    // size mismatch below.
    Eigen::VectorXd d_midpoint = segment.derivatives();
    if (d_midpoint.size() == 0) {
      d_midpoint = Eigen::VectorXd::Zero(n);
    }

    const Eigen::MatrixXd mass_matrix =
        drake::math::ExtractValue(mass_matrix_autodiff);
    const Eigen::VectorXd twice_mass_matrix_delta = 2.0 * (mass_matrix * delta);

    // delta = f(q~_1) - f(q~_0), hence the sign difference, and the midpoint
    // contributes half of its gradient to each endpoint.
    Scatter(-(jacobian_0.transpose() * twice_mass_matrix_delta) +
                0.5 * d_midpoint,
            i - 1);
    Scatter((jacobian_1.transpose() * twice_mass_matrix_delta) +
                0.5 * d_midpoint,
            i);

    q_tilde_0 = q_tilde_1;
    q_full_0 = q_full_1;
    jacobian_0 = jacobian_1;
  }

  (*y)(0) = drake::AutoDiffXd(scale_ * total_value, scale_ * total_gradient);
}
