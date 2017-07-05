#include "drake/automotive/idm_planner.h"

#include <cmath>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

static void ReplaceNanPartials(AutoDiffXd* x) {
  auto& derivs = (*x).derivatives();
  const int num_partials = derivs.size();
  for (int i{0}; i < num_partials; ++i) {
    if (isnan(derivs(i))) derivs(i) = 0.;
  }
}

static void ReplaceNanPartials(double* x) {}
static void ReplaceNanPartials(symbolic::Expression* x) {}

template <typename T>
const T IdmPlanner<T>::Evaluate(const IdmPlannerParameters<T>& params,
                                const T& ego_velocity, const T& target_distance,
                                const T& target_distance_dot) {
  DRAKE_DEMAND(params.IsValid());

  using std::pow;
  using std::sqrt;

  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();

  DRAKE_DEMAND(a > 0.);
  DRAKE_DEMAND(b > 0.);
  DRAKE_DEMAND(target_distance > 0.);

  // Compute the interaction acceleration terms.
  const T& closing_term =
      ego_velocity * target_distance_dot / T(2 * sqrt(a * b));
  const T& too_close_term = s_0 + ego_velocity * time_headway;

  T accel_interaction =
      pow((closing_term + too_close_term) / target_distance, 2.);
  ReplaceNanPartials(&accel_interaction);

  // Compute the free-road acceleration term.
  const T& accel_free_road = pow(ego_velocity / T(v_ref),
                                 ExtractDoubleOrThrow(delta));

  // Compute the resultant acceleration (IDM equation).
  return a * (1. - accel_free_road - accel_interaction);
}

// These instantiations must match the API documentation in idm_planner.h.
template class IdmPlanner<double>;
template class IdmPlanner<drake::AutoDiffXd>;
template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
