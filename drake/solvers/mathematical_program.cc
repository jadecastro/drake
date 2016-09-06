#include "mathematical_program.h"

#include "equality_constrained_qp_solver.h"
#include "ipopt_solver.h"
#include "linear_system_solver.h"
#include "moby_lcp_solver.h"
#include "nlopt_solver.h"
#include "optimization.h"
#include "snopt_solver.h"
#include "dreal_solver.h"

namespace drake {
namespace solvers {

namespace {

enum ProblemAttributes {
  kNoCapabilities = 0,
  kError = 1 << 0,  ///< Do not use, to avoid & vs. && typos.
  kGenericCost                     = 1 << 1,
  kGenericConstraint               = 1 << 2,
  kScalartypeConstraint            = 1 << 3,
  kQuadraticCost                   = 1 << 4,
  kQuadraticConstraint             = 1 << 5,
  kLinearCost                      = 1 << 6,
  kLinearConstraint                = 1 << 7,
  kLinearEqualityConstraint        = 1 << 8,
  kLinearComplementarityConstraint = 1 << 9
};
typedef uint32_t AttributesSet;

// TODO(ggould-tri) Refactor these capability advertisements into the
// solver wrappers themselves.

// Solver for simple linear systems of equalities
AttributesSet kLinearSystemSolverCapabilities = kLinearEqualityConstraint;

// Solver for equality-constrained QPs
AttributesSet kEqualityConstrainedQPCapabilities = (
  kQuadraticCost | kLinearCost | kLinearEqualityConstraint);

// Solver for Linear Complementarity Problems (LCPs)
AttributesSet kMobyLcpCapabilities = kLinearComplementarityConstraint;

// Solver for Quadratic Programs (QPs); commented out until Gurobi is ready
// to land.
// AttributesSet kGurobiCapabilities = (
//     kLinearEqualityConstraint | kLinearInequalityConstraint |
//     kLinearCost | kQuadraticCost);

// Solvers for generic systems of constraints and costs.
AttributesSet kGenericSolverCapabilities = (
    kGenericCost | kGenericConstraint |
    kQuadraticCost | kQuadraticConstraint |
    kLinearCost | kLinearConstraint | kLinearEqualityConstraint);

// Solvers for generic systems of constraints and costs.
AttributesSet kDrealSolverCapabilities = (
    kGenericConstraint | kScalartypeConstraint |
    kQuadraticCost | kQuadraticConstraint |
    kLinearCost | kLinearConstraint | kLinearEqualityConstraint);

// Returns true iff no capabilities are in required and not in available.
bool is_satisfied(AttributesSet required, AttributesSet available) {
  return ((required & ~available) == kNoCapabilities);
}

}  // anon namespace


MathematicalProgram::MathematicalProgram()
    : required_capabilities_(kNoCapabilities),
      ipopt_solver_(new IpoptSolver()),
      dreal_solver_(new DrealSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver()),
      linear_system_solver_(new LinearSystemSolver()),
      equality_constrained_qp_solver_(new EqualityConstrainedQPSolver()) {}

void MathematicalProgram::AddGenericCost() {
  required_capabilities_ |= kGenericCost;
}
void MathematicalProgram::AddGenericConstraint() {
  required_capabilities_ |= kGenericConstraint;
}
void MathematicalProgram::AddScalartypeConstraint() {
  required_capabilities_ |= kScalartypeConstraint;
}
void MathematicalProgram::AddQuadraticCost() {
  required_capabilities_ |= kQuadraticCost;
}
void MathematicalProgram::AddQuadraticConstraint() {
  required_capabilities_ |= kQuadraticConstraint;
}
void MathematicalProgram::AddLinearCost() {
  required_capabilities_ |= kLinearCost;
}
void MathematicalProgram::AddLinearConstraint() {
  required_capabilities_ |= kLinearConstraint;
}
void MathematicalProgram::AddLinearEqualityConstraint() {
  required_capabilities_ |= kLinearEqualityConstraint;
}
void MathematicalProgram::AddLinearComplementarityConstraint() {
  required_capabilities_ |= kLinearComplementarityConstraint;
}

SolutionResult MathematicalProgram::Solve(OptimizationProblem& prog) const {
  // This implementation is simply copypasta for now; in the future we will
  // want to tweak the order of preference of solvers based on the types of
  // constraints present.

  if (is_satisfied(required_capabilities_, kLinearSystemSolverCapabilities) &&
      linear_system_solver_->available()) {
    // TODO(ggould-tri) Also allow quadratic objectives whose matrix is
    // Identity: This is the objective function the solver uses anyway when
    // underconstrainted, and is fairly common in real-world problems.
    std::cerr << "linear system solver\n";
    return linear_system_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_,
                          kEqualityConstrainedQPCapabilities) &&
            equality_constrained_qp_solver_->available()) {
    std::cerr << "equality constrained solver\n";
    return equality_constrained_qp_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kMobyLcpCapabilities) &&
             moby_lcp_solver_->available()) {
    std::cerr << "moby lcp solver\n";
    return moby_lcp_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             snopt_solver_->available()) {
    std::cerr << "snopt solver\n";
    return snopt_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             ipopt_solver_->available()) {
    std::cerr << "ipopt solver\n";
    return ipopt_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             nlopt_solver_->available()) {
    std::cerr << "nlopt solver\n";
    return nlopt_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kDrealSolverCapabilities) &&
             dreal_solver_->available()) {
    std::cerr << "dreal solver\n";
    return dreal_solver_->Solve(prog);
  } else {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
