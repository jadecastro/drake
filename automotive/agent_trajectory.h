#pragma once

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

using multibody::SpatialVelocity;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using trajectories::PiecewisePolynomial;

class AgentTrajectory;

/// Wrapper for the raw trajectory values.
class TrajectoryAgentValues {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TrajectoryAgentValues)

  ///
  TrajectoryAgentValues(const PoseVector<double>& pose,
                        const FrameVelocity<double>& velocity) {
    values_.resize(PoseVector<double>::kSize + FrameVelocity<double>::kSize);
    values_.head(PoseVector<double>::kSize) = pose.CopyToVector();
    values_.tail(FrameVelocity<double>::kSize) = velocity.CopyToVector();
  }

  template <typename T>
  PoseVector<T> pose6() const {
    return {Eigen::Quaternion<T>{T{values_(0)}, T{values_(1)}, T{values_(2)},
                                 T{values_(3)}},
            Eigen::Translation<T, 3>{values_.segment(4, 3)}};
  }

  template <typename T>
  Eigen::Vector3<T> pose3() const {
    
  }

  template <typename T>
  FrameVelocity<T> velocity6() const {
    return FrameVelocity<T>{
        SpatialVelocity<T>{values_.tail(PoseVector<double>::kSize)}};
  }

  template <typename T>
  T speed() const {
    
  }

 private:
  explicit TrajectoryAgentValues(const Eigen::VectorXd& values)
      : values_(values) {
    DRAKE_DEMAND(values_.size() ==
                 PoseVector<double>::kSize + FrameVelocity<double>::kSize);
    // TODO: Make the quaternion ordering explicit w,x,y,z vs. x,y,z,w ?? (it's
    // resolved in TrajectoryAgent).
  }

  Eigen::VectorXd values_;

  friend class AgentTrajectory;
};

/// ** TODO ** Documentation
/// * Keeps the raw VectorX hidden from the TrajectoryAgent client.  All it sees
/// is TrajectoryAgentValues.
class AgentTrajectory {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectory)

  // TODO(jadecastro) Make the trajectory type configurable (e.g. Trajectory).
  // ** TODO ** Hide this as a private ctor?
  explicit AgentTrajectory(const PiecewisePolynomial<double>& trajectory)
      : trajectory_(trajectory) {
    // ** TODO: Checks.
  }

  /// Makes an AgentTrajectory from std::vectors of @p times, @p poses
  /// (PoseVector), and @p velocities (FrameVelocity) whose trajectory is a
  /// piecewise-cubic polynomial.
  // ** TODO ** Make the interpolation scheme configurable (e.g. FOH).
  static std::unique_ptr<AgentTrajectory> Make(
      const std::vector<double>& times,
      const std::vector<PoseVector<double>>& poses,
      const std::vector<FrameVelocity<double>>& velocities) {
    DRAKE_DEMAND(times.size() == poses.size());
    DRAKE_DEMAND(times.size() == velocities.size());
    std::vector<Eigen::MatrixXd> knots(times.size());
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      const TrajectoryAgentValues values(poses[i], velocities[i]);
      knots.emplace_back(
          values.values_);  // ** TODO: Is private member field access kosher?
    }
    return std::make_unique<AgentTrajectory>(
        PiecewisePolynomial<double>::Cubic(times, knots));
  }

  TrajectoryAgentValues value(double time) const {
    // N.B. Uses the private constructor of TrajectoryAgentValues.
    return TrajectoryAgentValues{trajectory_.value(time)};
  }

  // TODO(jadecastro) Decide if it makes sense to just expose trajectory_.
  bool empty() const { return trajectory_.empty(); }

  // TODO(jadecastro) Open up more PiecewisePolynomial/Trajectory accessors?

 private:
  // TODO: Resolve const here causing the compiler to complain about `attempt to
  // use a deleted function`.  Do we need Clone(), depsite being DEFAULT_COPY..
  // ?
  PiecewisePolynomial<double> trajectory_;
};

}  // namespace automotive
}  // namespace drake
