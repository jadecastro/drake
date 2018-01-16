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

Expression MakeCost(const Polynomial& p, Bounds b) {
  // Build a moment vector.
  MapType map = p.monomial_to_coefficient_map();
  Expression coeff;
  Variables ivars = p.indeterminates();
  for (const auto& monomial : map) {
    Variables vars = monomial.first.GetVariables();
    auto powers = monomial.first.get_powers();
    Expression moment{1.};
    for (const auto& var : ivars) {
      std::cout << "      power " << var << "  " << powers[var]+1 << std::endl;
      if (powers.find(var) == powers.end()) {
        moment *= b.second[var] - b.first[var];  // zero exponent case.
        continue;
      }
      moment *= (pow(b.second[var], powers[var]+1) -
                 pow(b.first[var], powers[var]+1)) / (powers[var]+1);
    }
    std::cout << " d var -- monomial -- moment: "
              << monomial.second  << "  " << monomial.first << "  " << moment << std::endl;
    coeff += moment * monomial.second;
  }
  return coeff;
}

/// Cubic polynomial system:
///   ẋ = 100 x³ - 25 x
template <typename T>
class CubicPolynomialSystemWithInput : public systems::VectorSystem<T> {
 public:
  CubicPolynomialSystemWithInput() : systems::VectorSystem<T>(0, 0) {
    // 0 inputs, 0 outputs, 1 continuous state.
    this->DeclareContinuousState(1);
  }

 private:
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    unused(context, input);
    (*derivatives)(0) = 100. * pow(state(0), 3.0) - 25. * state(0);
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
  const Variable& t = tvec(0);
  const Variable& x = xvec(0);

  const int d = 6;

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Domain bounds on t, x.
  const double T = 1.;
  const double x_bound = 5.;  // Bounding box on x: |x| ≤ x_bound.
  const Polynomial gt = Polynomial{t * (T - t)};
  const Polynomial gx1 = Polynomial{-(x - x_bound) * (x + x_bound)};

  // Terminal set.
  const Polynomial gxT{-(x - 0.1) * (x + 0.1)};

  // Inf  w'l  ∀ v, w, qᵢ, qT, q₀ᵢ, s₀ᵢ
  // s.t. Lv ≤ 0 on [0, T] × X
  //      v ≥ 0 on {T} × X_T
  //      w ≥ v + 1 on {0} × X
  //      w ≥ 0 on X
  const Polynomial v{prog.NewSosPolynomial({t, x}, d).first};
  const Polynomial w{prog.NewSosPolynomial({x}, d).first};

  // "L" operator.
  const Polynomial Lv{v.Jacobian(tvec).coeff(0) +
        v.Jacobian(xvec).coeff(0) * Polynomial((*derivatives)[0])};

  const Polynomial q0{prog.NewSosPolynomial({t, x}, d-2).first};
  const Polynomial q1{prog.NewSosPolynomial({t, x}, d-2).first};
  prog.AddSosConstraint(-Lv - q1 * gx1 - q0 * gt);

  const Polynomial qT{prog.NewSosPolynomial({x}, d-2).first};
  const Polynomial qtT{prog.NewSosPolynomial({t, x}, d-2).first};
  const Polynomial gtT = Polynomial{(t - T) * (T + 0.001 - t)};

  std::cout << " v: " << v << std::endl;
  std::cout << "   degree: " << v.TotalDegree() << std::endl;
  std::cout << "   dv: " << v.decision_variables() << std::endl;
  std::cout << "   ind: " << v.indeterminates() << std::endl;
  std::cout << " v subs: " << Polynomial{v.ToExpression().Substitute(t, 0.)} << std::endl;
  const Polynomial new_v(prog.NewFreePolynomial(v.ToExpression().Substitute(t, 0.));
  std::cout << "   degree: " << new_v.TotalDegree() << std::endl;
  std::cout << "   dv: " << new_v.decision_variables() << std::endl;
  std::cout << "   ind: " << new_v.indeterminates() << std::endl;
  prog.AddSosConstraint(new_v - qtT * gtT)
  // prog.AddSosConstraint(v - qT * gxT + qtT * gtT);

  const Polynomial q01{prog.NewSosPolynomial({x}, d-2).first};
  const Polynomial qt0{prog.NewSosPolynomial({t, x}, d-2).first};
  const Polynomial gt0 = Polynomial{-t * (t + 0.001)};
  prog.AddSosConstraint(w - v - 1. - q01 * gx1 + qt0 * gt0);

  const Polynomial s01{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(w - s01 * gx1);

  const Expression m = MakeCost(
      w, std::make_pair(Bound{{x, -x_bound}}, Bound{{x, x_bound}}));
  std::cout << " Cost function " << m << std::endl;
  prog.AddCost(m);

  const solvers::SolutionResult result{prog.Solve()};

  std::cout << " Solution result " << result << std::endl;
  //prog.PrintSolution();
  std::cout << " Program attributes " << std::endl;
  std::cout << "    positive semidefinite constraints " << prog.positive_semidefinite_constraints().size() << std::endl;
  std::cout << " Optimal cost " << prog.GetOptimalCost() << std::endl;

  std::cout << " w: " << w << std::endl;
  std::cout << "    Solution w: " << std::endl;
  for (const auto& it : w.decision_variables()) {
    std::cout << "               " << it << " " << prog.GetSolution(it) << std::endl;
  }

  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);
}

}  // namespace drake

int main() {
  drake::ComputeBackwardReachableSet();
  // drake::ComputeIndicator();
  return 0;
}
