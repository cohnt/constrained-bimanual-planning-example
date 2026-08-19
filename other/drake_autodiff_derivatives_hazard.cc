// A minimal reproduction of a lifetime hazard in drake::AutoDiffXd::derivatives().
//
// SUMMARY
//
// AutoDiff::derivatives() returns a DerivativesConstXpr, an Eigen expression
// that *views* the AutoDiff's internal storage rather than owning a copy.
// Separately, Eigen::Ref<const AutoDiffVecXd>::CoeffReturnType is a value, not
// a reference, so `x(i)` on such a Ref materializes a temporary AutoDiffXd.
//
// Together these mean that
//
//     const auto& d = x(i).derivatives();   // x is an Eigen::Ref<const ...>
//
// leaves `d` viewing storage inside a temporary AutoDiffXd that is destroyed at
// the end of the full-expression. Binding to `const auto&` extends the lifetime
// of the *expression object*, but not of the AutoDiffXd it points into. Reading
// `d` afterwards yields freed memory.
//
// This matters because `Eigen::Ref<const AutoDiffVecXd>& x` is exactly the
// signature of every Drake Constraint::DoEval / Cost::DoEval override, so
// `x(i).derivatives()` is a natural thing to write there. It fails silently:
// the values stay correct and only the gradients are corrupted, and only
// partially -- in the case that prompted this reproduction, 2 entries out of
// 16 were wrong, which passed every check except a finite-difference
// comparison of the gradient.
//
// Note that the same code is SAFE when the AutoDiffXd is a named object or an
// element of a concrete vector, since those yield real references. Only the
// temporary-producing forms dangle, which is what makes it easy to miss.
//
// BUILD AND RUN
//
//   g++ -std=c++20 -O2 -I${DRAKE_INSTALL_DIR}/include -I/usr/include/eigen3 \
//     drake_autodiff_derivatives_hazard.cc -o hazard \
//     -L${DRAKE_INSTALL_DIR}/lib -ldrake -Wl,-rpath,${DRAKE_INSTALL_DIR}/lib
//   ./hazard
//
// Reproduces at -O0, -O2 and -O3 with GCC 13.3 against Drake built from source.

#include <cstdio>
#include <type_traits>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"

namespace {

// Overwrites the stack that the destroyed temporary occupied, so that reading
// through a dangling view is visibly wrong rather than accidentally intact.
double Churn() {
  volatile double scratch[1024];
  for (int i = 0; i < 1024; ++i) scratch[i] = i * 1.5;
  double total = 0.0;
  for (int i = 0; i < 1024; ++i) total += scratch[i];
  return total;
}

void Show(const char* what, const Eigen::VectorXd& value, const char* want) {
  printf("  %-44s: %g %g %g %g   (want %s)\n", what, value[0], value[1],
         value[2], value[3], want);
}

}  // namespace

int main() {
  const int kSize = 4;
  Eigen::Matrix<drake::AutoDiffXd, Eigen::Dynamic, 1> v(kSize);
  for (int i = 0; i < kSize; ++i) {
    v(i) = drake::AutoDiffXd(i, Eigen::VectorXd::Unit(kSize, i));
  }
  const Eigen::Ref<const drake::AutoDiffVecXd> x = v;

  printf("Why the temporary appears:\n");
  printf("  element of a concrete vector is a reference : %d\n",
         static_cast<int>(std::is_reference_v<decltype(std::as_const(v)(1))>));
  printf("  element of an Eigen::Ref<const> is a reference: %d  <-- returns by value\n",
         static_cast<int>(std::is_reference_v<decltype(x(1))>));

  printf("\nSafe (the AutoDiffXd outlives the view):\n");
  {
    const auto& d = std::as_const(v)(1).derivatives();
    Churn();
    Show("named vector element, const auto&", d, "0 1 0 0");
  }

  printf("\nDangling (the AutoDiffXd is a temporary):\n");
  {
    // This is the shape that occurs inside Constraint::DoEval / Cost::DoEval.
    const auto& d = x(1).derivatives();
    Churn();
    Show("Eigen::Ref<const> element, const auto&", d, "0 1 0 0");
  }
  {
    const auto& d = (v(1) + v(2)).derivatives();
    Churn();
    Show("temporary from arithmetic, const auto&", d, "0 1 1 0");
  }
  {
    const auto& d = (2.0 * v)(1).derivatives();
    Churn();
    Show("element of a temporary expression", d, "0 2 0 0");
  }

  printf("\nThe fix in each case is to evaluate into a vector:\n");
  {
    const Eigen::VectorXd d = x(1).derivatives();
    Churn();
    Show("Eigen::Ref<const> element, VectorXd", d, "0 1 0 0");
  }
  {
    const Eigen::VectorXd d = (v(1) + v(2)).derivatives();
    Churn();
    Show("temporary from arithmetic, VectorXd", d, "0 1 1 0");
  }
  return 0;
}
