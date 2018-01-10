// A simple example of computing a backward reachable set for a polynomial
// dynamical system using occupation measures.
//
// "Convex computation of the region of attraction of polynomial control
// systems" by Didier Henrion and Milan Korda.
#include <cmath>
#include <ostream>

#include "drake/common/symbolic.h"
#include "drake/common/unused.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {

using std::cout;
using std::endl;
using std::pow;

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

using MapType = Polynomial::MapType;
using Bound = std::map<Variable, double>;
using Bounds = std::pair<Bound, Bound>;

Expression MakeMomentVector(const Polynomial& p, Bounds b) {
  // Build a moment vector.
  MapType map = p.monomial_to_coefficient_map();
  Expression coeff;
  for (const auto& monomial : map) {
    Variables vars = monomial.first.GetVariables();
    auto powers = monomial.first.get_powers();
    Expression moment{1.};
    for (const auto& var : vars) {
      moment *= (pow(b.second[var], powers[var]+1) -
                 pow(b.first[var], powers[var]+1)) / (powers[var]+1);
    }
    coeff += moment * monomial.second;
  }
  return coeff;
}

// TODO(jadecastro) Consider having this as a unit test?
// Solve Problem 4.1 from Henrion, Lasserre and Savorgnan: Approximate Volume
// and Integration for Basic Semi-Algebraic Sets,
// https://arxiv.org/pdf/0807.2505.pdf.
void ComputeIndicator() {
  // Set up the optimization problem.
  solvers::MathematicalProgram prog;
  const VectorX<Variable> xvec{prog.NewIndeterminates<1>("x")};
  const Variable& x = xvec(0);

  const int d = 10;

  // Domain bounds on x.
  const double x_bound = 1.;  // Bounding box on x: |x| ≤ x_bound.
  const Polynomial gx = Polynomial{x * (0.5 - x)};

  // Let K = [0, 0.5] = {x ∈ R : g₁(x) = x(0.5 − x) ≥ 0} define an interval
  // included in B = [−1, 1].
  //
  // Inf  w'l  ∀ v, w, q
  // s.t. w ≥ v + 1 on K
  const Polynomial v{prog.NewSosPolynomial({x}, d).first};
  const Polynomial w{prog.NewSosPolynomial({x}, d).first};
  const Polynomial q{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(w - v - 1. - q * gx);

  const Expression m = MakeMomentVector(
      w, std::make_pair(Bound{{x, -x_bound}}, Bound{{x, x_bound}}));
  // Expect the moments of the Lebesgue measure µ2 on B to be
  // y2 = (2, 0, 2/3, 0, 2/5, 0, 2/7, ...)
  std::cout << " Cost " << m << std::endl;
  prog.AddCost(m);

  const solvers::SolutionResult result{prog.Solve()};
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);
}

/// Cubic Polynomial System:
///   ẋ = -x + x³ + u
///   y = x
template <typename T>
class CubicPolynomialSystemWithInput : public systems::VectorSystem<T> {
 public:
  CubicPolynomialSystemWithInput() : systems::VectorSystem<T>(1, 0) {
    // 1 input, 0 outputs, 1 continuous state.
    this->DeclareContinuousState(1);
  }

 private:
  // ẋ = -x + x³ + u
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    unused(context, input);
    (*derivatives)(0) = -state(0) + pow(state(0), 3.0) + input(0);
  }
};

void ComputeBackwardReachableSet() {
  // Create the simple system.
  CubicPolynomialSystemWithInput<Expression> system;
  auto context = system.CreateDefaultContext();
  auto derivatives = system.AllocateTimeDerivatives();

  // Setup the optimization problem.
  solvers::MathematicalProgram prog;
  const VectorX<Variable> tvec{prog.NewIndeterminates<1>("t")};
  const VectorX<Variable> xvec{prog.NewIndeterminates<1>("x")};
  const VectorX<Variable> uvec{prog.NewIndeterminates<1>("u")};
  const Variable& t = tvec(0);
  const Variable& x = xvec(0);
  const Variable& u = uvec(0);

  const int d = 10;

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  context->FixInputPort(0, systems::BasicVector<Expression>::Make(u));
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Domain bounds on t, x, u.
  const double T = 2.;
  const double x_bound = 1.;  // Bounding box on x: |x| ≤ x_bound.
  const double u_bound = 1.;  // Bounding box on u: |u| ≤ u_bound.
  const Polynomial gt = Polynomial{t * (T - t)};
  const Polynomial gx1 = Polynomial{pow(x_bound, 2.)} - Polynomial{x * x};
  const Polynomial gu1 = Polynomial{pow(u_bound, 2.)} - Polynomial{u * u};

  // Terminal set.
  const Polynomial gxT{x * x};

  // Inf  w'l  ∀ v, w, qᵢ, qT, q₀ᵢ, s₀ᵢ
  // s.t. Av ≤ 0 on [0, T] × X × U
  //      v ≥ 0 on {T} × ∂X_T  (∂(⋅) denotes the boundary of (⋅))
  //      w ≥ v + 1 on {0} × X
  //      w ≥ 0 on X
  const Polynomial v{prog.NewFreePolynomial({t, x}, d)};
  const Polynomial w{prog.NewFreePolynomial({x}, d)};

  // "L" operator.
  const Polynomial Lv{v.Jacobian(tvec).coeff(0) +
        v.Jacobian(xvec).coeff(0) * Polynomial((*derivatives)[0])};

  const Polynomial q0{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial q1{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial r1{prog.NewSosPolynomial({t, x, u}, d-2).first};
  prog.AddSosConstraint(-Lv - q1 * gx1 - r1 * gu1 + q0 * gt);

  const Polynomial qT{prog.NewFreePolynomial({t, x, u}, d-2)};  // constraint at
                                                                // gxT == 0.
  const Polynomial qtT{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial gtT = Polynomial{(t - T) * (T + 0.001 - t)};
  prog.AddSosConstraint(v - qT * gxT + qtT * gtT);

  const Polynomial q01{prog.NewSosPolynomial({x}, d-2).first};
  const Polynomial qt0{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial gt0 = Polynomial{-t * (t + 0.001)};
  prog.AddSosConstraint(w - v - 1. - q01 * gx1 + qt0 * gt0);

  const Polynomial s01{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(w - s01 * gx1);

  const Expression m = MakeMomentVector(
      w, std::make_pair(Bound{{x, -x_bound}}, Bound{{x, x_bound}}));
  std::cout << " Cost " << m << std::endl;
  prog.AddCost(m);

  const solvers::SolutionResult result{prog.Solve()};
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);

  // cout << "Verified that " << v << " < " << prog.GetSolution(m.GetVariables()[0])
  //     << " is in the region of attraction." << endl;

  // Check that ρ ≃ 1.0.
  // DRAKE_DEMAND(std::abs(prog.GetSolution(rho) - 1.0) < 1e-6);
}
}  // namespace drake

int main() {
  // drake::ComputeBackwardReachableSet();
  drake::ComputeIndicator();
  return 0;
}
