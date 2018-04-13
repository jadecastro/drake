#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace automotive {

namespace internal {
static constexpr int kIndicesSize = 7;
}  // namespace internal

using multibody::SpatialVelocity;
using trajectories::PiecewisePolynomial;

class AgentTrajectory;

/// Wraps the raw data contained in a trajectory expressed in terms of pose and
/// velocitiy values.  Represents a translational and rotational transformation
/// of a reference frame A with respect to world frame W, expressed in x-y-z
/// coordinates, and translational and rotational velocities of frame A with
/// respect to W.
class AgentTrajectoryValues final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectoryValues)

  /// Default constructor.  Sets all poses to identity transforms, and all
  /// velocities to zero.
  AgentTrajectoryValues()
      : AgentTrajectoryValues(
            Eigen::Quaternion<double>::Identity(),
            Eigen::Translation<double, 3>::Identity(),
            SpatialVelocity<double>(Vector6<double>::Zero())) {}

  /// Fully-parameterized constructor.
  ///
  /// @param rotation the orientation R_WA of frame A with respect to the world
  /// frame W, and can be either normalized or unnormalized, but internally
  /// stores the normalized quantity.
  /// @param translation the x, y, z position p_WA of frame A measured from W's
  /// origin.
  /// @param velocity the (rotational/translational) spatial velocity Xdot_WA of
  /// the frame A with respect to frame W.
  AgentTrajectoryValues(const Eigen::Quaternion<double>& rotation,
                        const Eigen::Translation<double, 3>& translation,
                        const SpatialVelocity<double>& velocity)
      : rotation_(rotation), translation_(translation), velocity_(velocity) {
    rotation_.normalize();
  }

  /// Accesses X_WA, an Isometry3 representation of pose of A, expressed in
  /// world-frame coordinates W.
  Eigen::Isometry3d isometry3() const {
    Eigen::Isometry3d isometry(translation_);
    isometry.rotate(rotation_);
    return isometry;
  }

  /// Accesses the projection of the pose of frame A to the x-y components of
  /// translation and the z-component of rotation.
  Eigen::Vector3d pose3() const {
    const double w_z = math::QuaternionToSpaceXYZ(rotation_).z();
    return Eigen::Vector3d{translation_.x(), translation_.y(), w_z};
  }

  /// Accesses Xdot_WA, a SpatialVelocity of frame A, expressed in world-frame
  /// coordinates W.
  const SpatialVelocity<double>& velocity() const { return velocity_; }

  /// Gets the speed `‖pdot_WA‖₂`, the 2-norm of the world-frame translational
  /// velocities.
  double speed() const { return velocity_.translational().norm(); }

 private:
  // Constructs AgentTrajectoryValues from `raw_pose`, a VectorXd of raw pose
  // values containing translations and quaternions of frame A wrt. W, and
  // `raw_velocity`, a VectorXd of raw velocity values containing
  // time-derivatives of translations and quaternions of frame A wrt. W.
  explicit AgentTrajectoryValues(const Eigen::VectorXd raw_pose,
                                 const Eigen::VectorXd raw_velocity)
      : rotation_(Eigen::Quaternion<double>(
            raw_pose(Indices::kRw), raw_pose(Indices::kRx),
            raw_pose(Indices::kRy), raw_pose(Indices::kRz))),
        translation_(Eigen::Translation<double, 3>(raw_pose(Indices::kTx),
                                                   raw_pose(Indices::kTy),
                                                   raw_pose(Indices::kTz))) {
    DRAKE_DEMAND(raw_pose.size() == internal::kIndicesSize);
    DRAKE_DEMAND(raw_velocity.size() == internal::kIndicesSize);

    // ** TODO ** Needed vs. just normalizing? Also: Add unit tests for the case
    // where quat is unnormalized, or else add this check to all ctors and state
    // in the documentation that it must be.
    DRAKE_DEMAND(math::IsQuaternionValid(
        rotation_, std::numeric_limits<double>::epsilon()));

    const Eigen::Vector4d quat_dot(
        raw_velocity(Indices::kRw), raw_velocity(Indices::kRx),
        raw_velocity(Indices::kRy), raw_velocity(Indices::kRz));
    const Eigen::Vector3d w =
        math::CalculateAngularVelocityExpressedInBFromQuaternionDt(rotation_,
                                                                   quat_dot);
    // N.B. Frame `B` above is our world coordinate system `W`.
    const Eigen::Vector3d v(raw_velocity(Indices::kTx),
                            raw_velocity(Indices::kTy),
                            raw_velocity(Indices::kTz));
    velocity_ = SpatialVelocity<double>(w, v);
  }

  // Accesses the raw pose data in the form of a VectorXd in the order
  // enumerated within Indices.
  Eigen::VectorXd to_raw_pose() const {
    Eigen::VectorXd vector(internal::kIndicesSize);
    vector(Indices::kTx) = translation_.x();
    vector(Indices::kTy) = translation_.y();
    vector(Indices::kTz) = translation_.z();
    vector(Indices::kRw) = rotation_.w();
    vector(Indices::kRx) = rotation_.x();
    vector(Indices::kRy) = rotation_.y();
    vector(Indices::kRz) = rotation_.z();
    return vector;
  }

  // Accesses the raw velocity data in the form of a VectorXd in the order
  // enumerated within Indices.
  Eigen::VectorXd to_raw_velocity() const {
    Eigen::VectorXd vector(internal::kIndicesSize);
    vector(Indices::kTx) = velocity_.translational().x();
    vector(Indices::kTy) = velocity_.translational().y();
    vector(Indices::kTz) = velocity_.translational().z();

    Eigen::Vector4d quat_dot =
        math::CalculateQuaternionDtFromAngularVelocityExpressedInB(
            rotation_, velocity_.rotational());
    // N.B. Frame `B` above is our world coordinate system `W`.
    vector(Indices::kRw) = quat_dot[0];
    vector(Indices::kRx) = quat_dot[1];
    vector(Indices::kRy) = quat_dot[2];
    vector(Indices::kRz) = quat_dot[3];
    return vector;
  }

  // Enumerates the indices for each of the raw pose data passed privately
  // to/from this class.
  struct Indices {
    static const int kTx = 0;
    static const int kTy = 1;
    static const int kTz = 2;
    static const int kRw = 3;
    static const int kRx = 4;
    static const int kRy = 5;
    static const int kRz = 6;
  };

  Eigen::Quaternion<double> rotation_;
  Eigen::Translation<double, 3> translation_;
  SpatialVelocity<double> velocity_;

  // We friend AgentTrajectory to allow it to populate via the raw data vector
  // constructor and raw data accessor.
  friend class AgentTrajectory;
};

/// An identifier for the type of interpolation to be used when forming an
/// AgentTrajectory.  These types mirror the associated constructors for
/// PiecewisePolynomial (see common/trajectories/piecewise_polynomial.h for
/// details on each of the interpolation schemes).
enum class InterpolationType {
  kZeroOrderHold,
  kFirstOrderHold,
  kCubic,
  kPchip
};

/// A class that wraps a piecewise trajectory instantiated based on pose data
/// and, optionally, velocity data.  Pose data can be either paired with time
/// data, or forward speed data.  The underlying piecewise polynomial may be
/// freely configured and, when evaluated at each knot point, the result
/// preserves the input data exactly.
class AgentTrajectory final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AgentTrajectory)

  /// Makes an AgentTrajectory from a discrete set of time-indexed
  /// AgentTrajectoryValues data containing both pose and velocity, under the
  /// specified interpolation scheme.
  ///
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory.
  /// @param times a vector of time values representing the break points of the
  /// trajectory.
  /// @param knots a knot points containing a vector of AgentTrajectoryValues,
  /// whose length must match that of @p times.
  /// @throws std::logic_error if @p interp_type is not supported.
  static AgentTrajectory Make(const InterpolationType& interp_type,
                              const std::vector<double>& times,
                              const std::vector<AgentTrajectoryValues>& knots) {
    DRAKE_THROW_UNLESS(times.size() == knots.size());
    std::vector<Eigen::MatrixXd> pose_knots_vector(times.size());
    std::vector<Eigen::MatrixXd> velocity_knots_vector(times.size());
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      pose_knots_vector[i] = knots[i].to_raw_pose();
      velocity_knots_vector[i] = knots[i].to_raw_velocity();
    }
    const PiecewisePolynomial<double> pose_traj =
        MakePiecewisePolynomial(interp_type, times, pose_knots_vector);
    const PiecewisePolynomial<double> velocity_traj =
        MakePiecewisePolynomial(interp_type, times, velocity_knots_vector);
    return AgentTrajectory(pose_traj, velocity_traj);
  }

  /// Makes an AgentTrajectory from a discrete set of time-indexed pose data,
  /// under the specified interpolation scheme.  The underlying velocity
  /// trajectory is inferred from the pose data based on time-differentiation of
  /// the underlying piecewise polynomial trajectory.  Beware that this is
  /// potentially lossy, since derivatives are taken from poses.
  ///
  /// @param interp_type an InterpolationType with the interpolation scheme used
  /// for building a piecewise polynomial trajectory.
  /// @param times a vector of time values representing the break points of the
  /// trajectory.
  /// @param knots a knot points containing a pose vector of Isometry3d, whose
  /// length must match that of @p times.
  /// @throws std::logic_error if @p interp_type is not supported.
  static AgentTrajectory Make(const InterpolationType& interp_type,
                              const std::vector<double>& times,
                              const std::vector<Eigen::Isometry3d>& knots) {
    DRAKE_THROW_UNLESS(times.size() == knots.size());
    std::vector<Eigen::MatrixXd> knots_vector(times.size());
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      const Eigen::Translation<double, 3>& p{knots[i].translation().x(),
                                             knots[i].translation().y(),
                                             knots[i].translation().z()};
      const Eigen::Quaternion<double>& q =
          math::RotationMatrix<double>(knots[i].rotation()).ToQuaternion();
      const AgentTrajectoryValues poses(
          q, p, SpatialVelocity<double>(Vector6<double>::Zero()));
      knots_vector[i] = poses.to_raw_pose();
    }
    const PiecewisePolynomial<double> pose_traj =
        MakePiecewisePolynomial(interp_type, times, knots_vector);
    const PiecewisePolynomial<double> velocity_traj = pose_traj.derivative();
    return AgentTrajectory(pose_traj, velocity_traj);
  }

  /// Makes an AgentTrajectory from a discrete set of (time-independent)
  /// waypoints, based on a vector of speeds.  The resulting trajectory is
  /// assumed to start at time time t = 0 and have linearly-interpolated poses
  /// (line-of-sight connectivity between waypoints) and linearly-interpolated
  /// speeds, modeling piecewise-constant accelerations between all waypoints.
  ///
  /// @param waypoints a vector of waypoints containing a pose vector of
  /// Isometry3d, whose length must match that of @p speeds.
  /// @param speeds a vector of speeds to be applied at knot points and linearly
  /// interpolated between waypoints.  All entries of @p speeds must be ≥ 0 and
  /// (with the exception of the last entry) any zero entries must be followed
  /// by a nonzero entry to avoid deadlock.
  static AgentTrajectory MakeFromWaypoints(
      const std::vector<Eigen::Isometry3d>& waypoints,
      const std::vector<double>& speeds) {
    DRAKE_THROW_UNLESS(speeds.size() == waypoints.size());
    std::vector<double> times(speeds.size());
    times[0] = 0.;

    for (int i{0}; i < static_cast<int>(speeds.size()); i++) {
      DRAKE_DEMAND(speeds[i] >= 0.);
      if (speeds[i] != speeds.back()) {
        // speedₖ == 0. ⇒ speedₖ₊₁ > 0., ∀ k = 0..N-1
        DRAKE_DEMAND((speeds[i + 1] > 0.) || (speeds[i] != 0.));

        // Populate the vector of breaks based on the assumption that path
        // length of two points is just their Euclidean distance.
        const double distance =
            (waypoints[i].translation() - waypoints[i + 1].translation())
                .norm();
        const double delta_t = 2 * distance / (speeds[i + 1] + speeds[i]);
        times[i + 1] = delta_t + times[i];
      }
    }
    return Make(InterpolationType::kFirstOrderHold, times, waypoints);
  }

  /// Makes an AgentTrajectory from a discrete set of (time-independent)
  /// waypoints, based on a constant speed.  The resulting trajectory is assumed
  /// to have linearly-interpolated poses (line-of-sight connectivity between
  /// waypoints).
  ///
  /// @param speeds a constant (nonzero) speed to be applied over the entirety
  /// of the trajectory.
  /// @param waypoints a vector of waypoints containing a pose vector of
  /// Isometry3d.
  static AgentTrajectory MakeFromWaypoints(
      const std::vector<Eigen::Isometry3d>& waypoints, double speed) {
    DRAKE_DEMAND(speed > 0.);
    std::vector<double> speeds(waypoints.size());
    std::fill(speeds.begin(), speeds.end(), speed);
    return MakeFromWaypoints(waypoints, speeds);
  }

  /// Evaluates the AgentTrajectory at a given @p time, with the result returned
  /// as AgentTrajectoryValues.
  AgentTrajectoryValues value(double time) const {
    return AgentTrajectoryValues{pose_trajectory_.value(time),
                                 velocity_trajectory_.value(time)};
  }

 private:
  // Constructs an AgentTrajectory from an appropriately-dimensioned
  // PiecewisePolynomial, given by @p trajectory.
  explicit AgentTrajectory(
      const PiecewisePolynomial<double>& pose_trajectory,
      const PiecewisePolynomial<double>& velocity_trajectory)
      : pose_trajectory_(pose_trajectory),
        velocity_trajectory_(velocity_trajectory) {}

  // Constructs a PiecewisePolynomial from the provided vectors of @p times
  // (breaks) and @p knots, with interpolation scheme chosen according to @p
  // interp_type.
  static PiecewisePolynomial<double> MakePiecewisePolynomial(
      const InterpolationType& interp_type, const std::vector<double>& times,
      const std::vector<Eigen::MatrixXd>& knots) {
    switch (interp_type) {
      case InterpolationType::kZeroOrderHold:
        return PiecewisePolynomial<double>::ZeroOrderHold(times, knots);
      case InterpolationType::kFirstOrderHold:
        return PiecewisePolynomial<double>::FirstOrderHold(times, knots);
      case InterpolationType::kCubic:
        return PiecewisePolynomial<double>::Cubic(times, knots);
      case InterpolationType::kPchip:
        return PiecewisePolynomial<double>::Pchip(times, knots);
      default:
        throw std::logic_error("The provided interp_type is not supported.");
    }
  }

  PiecewisePolynomial<double> pose_trajectory_;
  PiecewisePolynomial<double> velocity_trajectory_;
};

}  // namespace automotive
}  // namespace drake
