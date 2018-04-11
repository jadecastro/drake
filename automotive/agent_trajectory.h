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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgentValues)

  ///
  TrajectoryAgentValues(const PoseVector<double>& pose,
                        const FrameVelocity<double>& velocity)
   {
    // ** TODO ** Compare against storing just an Eigen::VectorXd of values.  Is
    // this too memory hungry?
  }

  /// Create a 6D pose with respect to the world frame.
  // ** TODO ** Body- vs. World-frame notation.
  PoseVector<double>& pose6_WA() const { return *pose_; }

  // ** TODO ** Body- vs. World-frame notation.
  /*
  const Eigen::Vector3d pose3_WA() const {
    Eigen::Vector2d planar_pose{pose6_WA().get_translation().x(),
          pose6_WA().get_translation().y()};
    double w_z = pose6_WA().get_rotation().z();
    return Eigen::Vector3d{0., 1., 2.};
  }
  */

  // ** TODO ** Body- vs. World-frame notation.
  FrameVelocity<double>& velocity6_WA() const { return *velocity_; }

  // ** TODO ** Body- vs. World-frame notation.
  /*
  Eigen::Vector3d velocity3_WA() const {
    const Eigen::Vector2d planar_velocity =
        velocity6_WA().get_velocity().translational().head(2);
    return {velocity6_WA().get_velocity().rotational().z(), planar_velocity};
  }
  */

  // ** TODO ** Do we need accessors in body frame also/instead?

  double speed() const {
    return velocity6_WA().get_velocity().translational().norm();
  }

 private:
  // Values are expected to be in the following order:
  // {x, y, z, qw, qx, qy, qz, ωx, ωy, ωz, vx, vy, vz}
  // ** TODO ** Split this up into four params?
  explicit TrajectoryAgentValues(const Eigen::VectorXd& values) {
    DRAKE_DEMAND(values.size() ==
                 PoseVector<double>::kSize + FrameVelocity<double>::kSize);
    pose_->set_translation(Eigen::Translation<double, 3>(values.head(3)));
    const Eigen::Vector4d q = values.segment(3, 4);
    pose_->set_rotation(Eigen::Quaternion<double>(q(0), q(1), q(2), q(3)));
    const Eigen::Vector3d w = values.segment(PoseVector<double>::kSize, 3);
    const Eigen::Vector3d v = values.tail(3);
    const SpatialVelocity<double> velocity(w, v);
    velocity_->set_velocity(velocity);
    // TODO: Make the quaternion ordering explicit w,x,y,z vs. x,y,z,w ?? (it's
    // resolved in TrajectoryAgent, but document above).
  }

  // Values populate VectorXd as per the following order:
  // {x, y, z, qw, qx, qy, qz, ωx, ωy, ωz, vx, vy, vz}
  Eigen::VectorXd values() const {
    Eigen::VectorXd vector;
    vector << pose_->CopyToVector();
    vector << velocity_->CopyToVector();
    return vector;
  }

  std::unique_ptr<PoseVector<double>> pose_;
  std::unique_ptr<FrameVelocity<double>> velocity_;

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
      // ** TODO ** Re-think this part.  Perhaps a make_knot() in
      // TrajectoryAgentValues?
      const TrajectoryAgentValues values(poses[i], velocities[i]);
      knots.emplace_back(values.values());
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
