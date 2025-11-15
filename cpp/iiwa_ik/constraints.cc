#include <Eigen/Dense>

#include "drake/solvers/mathematical_program.h"

using namespace drake;

class IiwaBimanualReachableConstraint final : public solvers::Constraint {
 public:
  IiwaBimanualReachableConstraint(bool shoulder_up, bool elbow_up,
                                  bool wrist_up)
      : solvers::Constraint(4,  // number of constraint equations
                            8,  // dimension of q_and_psi
                            Eigen::Vector4d::Constant(-1.0),  // lower bounds
                            Eigen::Vector4d::Constant(1.0)),  // upper bounds
        shoulder_up_(shoulder_up),
        elbow_up_(elbow_up),
        wrist_up_(wrist_up) {
    set_is_thread_safe(true);
  }

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const Eigen::VectorX<T>>& q,
                     Eigen::VectorX<T>* y) const {
    Eigen::VectorX<T> unclipped;
    IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_,
                                    &unclipped);
    *y = unclipped;  // should be length 4
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric<double>(q, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& q,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric<AutoDiffXd>(q, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& q,
              VectorX<symbolic::Expression>* y) const override {
    DoEvalGeneric<symbolic::Expression>(q, y);
  }

  bool shoulder_up_{}, elbow_up_{}, wrist_up_{};
};

class IiwaBimanualJointLimitConstraint final
    : public drake::solvers::Constraint {
 public:
  IiwaBimanualJointLimitConstraint(const Eigen::VectorXd& lower_bound,
                                   const Eigen::VectorXd& upper_bound,
                                   bool shoulder_up, bool elbow_up,
                                   bool wrist_up)
      : drake::solvers::Constraint(lower_bound.size(),  // number of outputs
                                   8,  // dimension of q_and_psi
                                   lower_bound, upper_bound),
        shoulder_up_(shoulder_up),
        elbow_up_(elbow_up),
        wrist_up_(wrist_up) {
    set_is_thread_safe(true);
  }

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const Eigen::VectorX<T>>& q,
                     Eigen::VectorX<T>* y) const {
    // We only have to worry about the subordinate arm's joint limits.
    *y = IiwaBimanualParameterization<T>(q, shoulder_up_, elbow_up_, wrist_up_,
                                         nullptr)
             .tail(7);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric<double>(q, y);
  }

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override {
    DoEvalGeneric<drake::AutoDiffXd>(q, y);
  }

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
      drake::VectorX<drake::symbolic::Expression>* y) const override {
    DoEvalGeneric<drake::symbolic::Expression>(q, y);
  }

  bool shoulder_up_{}, elbow_up_{}, wrist_up_{};
};