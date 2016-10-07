#include <vector>

#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace systems {

/// HybridContinuousState is a ContinuousState that switches among different
/// ContinuousStates.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class HybridContinuousState : public ContinuousState<T> {
 public:
  /// Constructs a ContinuousState that is composed of other ContinuousStates,
  /// which are not owned by this object and must outlive it. Some of the
  /// subsystem states may be nullptr if the system is stateless.
  explicit HybridContinuousState(std::vector<ContinuousState<T>*> substates)
       substates_(std::move(substates)) {}

  ~HybridContinuousState() override {}

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  const ContinuousState<T>* get_substate(int index) const {
    int num_substates = static_cast<int>(substates_.size());
    DRAKE_DEMAND(index >= 0 && index < num_substates);
    return substates_[index];
  }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  ContinuousState<T>* get_mutable_substate(int index) {
    int num_substates = static_cast<int>(substates_.size());
    DRAKE_DEMAND(index >= 0 && index < num_substates);
    return substates_[index];
  }

 private:
  std::vector<ContinuousState<T>*> substates_;
};

}  // namespace systems
}  // namespace drake
