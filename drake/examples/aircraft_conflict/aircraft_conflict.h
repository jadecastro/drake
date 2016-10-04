#pragma once

#include <memory>

namespace drake {
namespace aircraft_conflict {

/// Canonical example hybrid dynamical system representation of a conflict
/// resolution scenario for two airplanes.
///
/// Taken from:
/// Claire Tomlin. Hybrid Systems with Application to Air Traffic Management.
///   PhD thesis, Department of Electrical Engineering, University of
///   California, Berkeley, 1998
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// To use other specific scalar types see bouncing_ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in drakeBouncingBall.
///
/// - Inputs: (no inputs).
/// - States: Relative linear and rotational positions and velocities
///           between the two airplanes in a planar coordinate system.
///           linear position, velocity are in units m, m/s;
///           rotational position, velocity are in units rad, rad/s.
template <typename T>
class AircraftConflictResolution : public HybridAutomaton<T> {
 public:
  // Constructor.
  AircraftConflictResolution();

  //
  T EvalGuard(const systems::Context<T>& context) const;

  //
  void PerformReset(systems::Context<T>* context) const;

  //
  void 

 private:
  const double max_velocity_meters_per_sec_ = 1.0;  // maximum allowed velocity.
  const double min_velocity_meters_per_sec_ = -1.0;  // minimum allowed
    // velocity.

};

}  // namespace aircraft_conflict
}  // namespace drake
