// A simple example of computing a region of attraction for a (polynomial)
// dynamical system. Note that this is a C++ version of
// CubicPolynomialExample.m which is available at the following link:
//
// https://github.com/RobotLocomotion/drake/blob/00ec5a9836871d5a22579963d45376b5979e41d5/drake/examples/CubicPolynomialExample.m.
//
// TODO(russt): Provide an additional python-only implementation of this
// example.

#include <cmath>
#include <ostream>

#include "drake/common/symbolic.h"
#include "drake/common/unused.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {

using std::cout;
using std::endl;

using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;

/// Cubic Polynomial System:
///   ẋ = -x + x³ + u
///   y = x
template <typename T>
class CubicPolynomialSystemWithInput : public systems::VectorSystem<T> {
 public:
  CubicPolynomialSystem()
      : systems::VectorSystem<T>(1, 0) {  // One input, zero outputs.
    this->DeclareContinuousState(1);      // One state variable.
  }

 private:
  // ẋ = -x + x³ + u
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    unused(context, input);
    using std::pow;
    (*derivatives)(0) = -state(0) + pow(state(0), 3.0) + input(0);
  }
};

void ComputeBackwardReachableSet() {
  // Create the simple system.
  CubicPolynomialSystem<Expression> system;
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

  const Polynomial xT{x * x};  // Terminal set.

  const int d = 10;
  const double T = 2.;

  const double xBound = 1.;  // Bounding box on x: |x| \leq xBound.
  const double uBound = 1.;  // Bounding box on u: |u| \leq uBound.

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  context->FixInputPort(0, u);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Define the function v.
  const Polynomial v{{t, x}, d};
  const Polynomial w{x, d};


  gx1 = Polynomial{xBound^2} - Polynomial{x * x};
  gu1 = Polynomial{uBound^2} - Polynomial{u * u};

  const Polynomial Lv{v.Jacobian(t).coeff(0) +
        v.Jacobian(xvec).coeff(0) * Polynomial((*derivatives)[0])};

  // Inf  w'*l
  // s.t. Av \leq 0 on [0, T] \cross X \cross U
  //      v \geq 0 on {T} \cross X_T
  //      w \geq v + 1 on {0} \cross X
  //      w \geq 0 on X
  const Variable rho{prog.NewContinuousVariables<1>("rho").coeff(0)};
  const Polynomial rho_poly{rho, {} /* no indeterminate */};

  const Polynomial q0{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial q1{prog.NewSosPolynomial({t, x, u}, d-2).first};
  const Polynomial q2{prog.NewSosPolynomial({t, x, u}, d-2).first};
  prog.AddSosConstraint(-Lv - q1 * gx1 - r1 * gu1 + Polynomial{t*(T - t)} * q0);

  prog.

  prog.AddCost();

  const solvers::SolutionResult result{prog.Solve()};
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);

  cout << "Verified that " << V << " < " << prog.GetSolution(rho)
       << " is in the region of attraction." << endl;

  // Check that ρ ≃ 1.0.
  DRAKE_DEMAND(std::abs(prog.GetSolution(rho) - 1.0) < 1e-6);
}
}  // namespace drake

int main() {
  drake::ComputeBackwardReachableSet();
  return 0;
}
