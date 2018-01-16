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
    std::cout << " coeff -- monomial -- moment: "
              << monomial.second  << "  " << monomial.first << "  " << moment << std::endl;
    coeff += moment * monomial.second;
  }
  return coeff;
}

Polynomial SubstituteIndeterminate(const Polynomial& p,
                                   const Environment& env) {
  const MapType map = p.monomial_to_coefficient_map();
  Polynomial p_new;
  for (const auto& monom : map) {
    const auto monom_coeff = monom.first.Substitute(env);
    const symbolic::Monomial new_monom{monom_coeff.second};
    // const symbolic::Monomial new_monom{monom.first};
    p_new.AddProduct(monom.second * monom_coeff.first, new_monom);
  }
  return p_new;
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

  const int d = 10;

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Domain bounds on t, x.
  const double T = 1.;
  const double x_bound = 1.;  // Bounding box on x: |x| ≤ x_bound.

  const Polynomial gt = Polynomial{t * (T - t)};
  const Polynomial gx1 = Polynomial{-(x - x_bound) * (x + x_bound)};

  // Terminal set.
  const Polynomial gxT{-(x - 0.1) * (x + 0.1)};

  // Inf  w'l  ∀ v, w, qᵢ, qT, q₀ᵢ, s₀ᵢ
  // s.t. Lv ≤ 0 on [0, T] × X
  //      v ≥ 0 on {T} × X_T
  //      w ≥ v + 1 on {0} × X
  //      w ≥ 0 on X
  const Polynomial v{prog.NewFreePolynomial({t, x}, d)};
  const Polynomial w{prog.NewFreePolynomial({x}, d)};
  std::cout << " num w " << w.decision_variables().size() << std::endl;
  std::cout << " num v " << v.decision_variables().size() << std::endl;

  // "L" operator.
  const Polynomial Lv{v.Jacobian(tvec).coeff(0) +
        v.Jacobian(xvec).coeff(0) * Polynomial((*derivatives)[0])};

  const Polynomial q1{prog.NewSosPolynomial({t, x}, d-2).first};
  const Polynomial qt{prog.NewSosPolynomial({t, x}, d-2).first};
  prog.AddSosConstraint(-Lv - q1 * gx1 - qt * gt);
  std::cout << " num q1 " << q1.decision_variables().size() << std::endl;
  std::cout << " num qt " << qt.decision_variables().size() << std::endl;

  const Polynomial v_at_T = SubstituteIndeterminate(v, Environment{{t, T}});
  const Polynomial qT{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(v_at_T - qT * gxT);
  std::cout << " num qT " << qT.decision_variables().size() << std::endl;

  const Polynomial v_at_0 = SubstituteIndeterminate(v, Environment{{t, 0.}});
  const Polynomial q01{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(w - v_at_0 - 1. - q01 * gx1);
  std::cout << " num q01 " << q01.decision_variables().size() << std::endl;

  const Polynomial s01{prog.NewSosPolynomial({x}, d-2).first};
  prog.AddSosConstraint(w - s01 * gx1);
  std::cout << " num s01 " << s01.decision_variables().size() << std::endl;

  const Expression m = MakeCost(
      w, std::make_pair(Bound{{x, -x_bound}}, Bound{{x, x_bound}}));
  std::cout << " Cost function " << m << std::endl;
  prog.AddCost(m);

  const solvers::SolutionResult result{prog.Solve()};

  std::cout << " Solution result " << result << std::endl;
  //prog.PrintSolution();
  std::cout << " Program attributes " << std::endl;
  std::cout << "    number of decision variables " << prog.num_vars() << std::endl;
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
