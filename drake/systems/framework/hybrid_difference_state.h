#include <vector>

#include "drake/systems/framework/difference_state.h"

namespace drake {
namespace systems {

/// HybridDifferenceState is a DifferenceState that switches among different
/// DifferenceStates.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class HybridDifferenceState : public DifferenceState<T> {
 public:
  /// Constructs a DifferenceState that is composed of other DifferenceStates,
  /// which are not owned by this object and must outlive it. Some of the
  /// subsystem states may be nullptr if the system is stateless.
  explicit HybridDifferenceState(std::vector<DifferenceState<T>*> substates)
       substates_(std::move(substates)) {}

  ~HybridDifferenceState() override {}

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  const DifferenceState<T>* get_substate(int index) const {
    int num_substates = static_cast<int>(substates_.size());
    DRAKE_DEMAND(index >= 0 && index < num_substates);
    return substates_[index];
  }

  /// Returns the continuous state at the given @p index, or nullptr if that
  /// system is stateless. Aborts if @p index is out-of-bounds.
  DifferenceState<T>* get_mutable_substate(int index) {
    int num_substates = static_cast<int>(substates_.size());
    DRAKE_DEMAND(index >= 0 && index < num_substates);
    return substates_[index];
  }

 private:
  std::vector<DifferenceState<T>*> substates_;
};

}  // namespace systems
}  // namespace drake
