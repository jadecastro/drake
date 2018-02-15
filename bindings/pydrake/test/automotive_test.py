#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.automotive import (
    LaneDirection,
    IdmController,
    PurePursuitController,
    RoadPositionStrategy,
    ScanStrategy,
    SimpleCar,
    create_lane_direction,
    create_driving_command,
    )
from pydrake.maliput.api import (
    RoadGeometryId,
    )
from pydrake.maliput.dragway import (
    create_dragway,
    )
from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.primitives import (
    Multiplexer,
    TrajectorySource,
    )
from pydrake.systems.rendering import (
    PoseAggregator,
    )
from pydrake.trajectories import (
    PiecewisePolynomialTrajectory,
    create_zoh_piecewise_polynomial,
    create_foh_piecewise_polynomial,
    )


# Construct a IDM-controlled SimpleCar in a dragway, a middle-ground between IDM
# and MOBIL.  Essentially it's a redeaux of
# AutomotiveSimulator::IdmControlledCar.
class TestAutomotiveDiagram(unittest.TestCase):
    def test_idm_dragway(self):
        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("single-lane road")
        rg = create_dragway(rg_id, 2, 100., 4., 0., 1., 1e-6, 1e-6)
        segment = rg.junction(0).segment(0)
        lane_0 = segment.lane(0)
        lane_1 = segment.lane(1)

        # Create a trajectory source system from some made-up trajectory data.
        # (Note that we're not using TrajectoryCar, as it doesn't track velocity
        # profiles.)
        # TODO import NGSIM data.
        breaks = [0., 1.]
        knots = []
        knots.append(np.array([5., 34.]))
        knots.append(np.array([79., 42.]))
        traject = create_foh_piecewise_polynomial(breaks, knots)
        source = TrajectorySource(PiecewisePolynomialTrajectory(traject))
        # TODO(jadecastro) Create PoseVector and FrameVelocity from this source;
        # tie into PoseAggregator.

        # Build a diagram with the IDM car placed in lane 0.
        builder = framework.DiagramBuilder()

        # TODO: used named variables
        idm = builder.AddSystem(IdmController(
            rg, ScanStrategy.kPath, RoadPositionStrategy.kExhaustiveSearch, 0.))
        simple_car = builder.AddSystem(SimpleCar())
        pursuit = builder.AddSystem(PurePursuitController())
        mux = builder.AddSystem(Multiplexer(create_driving_command()))
        aggregator = builder.AddSystem(PoseAggregator())

        # TODO(jadecastro) Use named port getters provided by each system.
        builder.Connect(simple_car.get_output_port(1), idm.get_input_port(0))
        builder.Connect(simple_car.get_output_port(2), idm.get_input_port(1))
        builder.Connect(pursuit.get_output_port(0), mux.get_input_port(0))
        builder.Connect(idm.get_output_port(0), mux.get_input_port(1))
        builder.Connect(mux.get_output_port(0), simple_car.get_input_port(0))

        ports = aggregator.AddSinglePoseAndVelocityInput("idm_car_0", 0)

        builder.Connect(simple_car.get_output_port(1), ports[0])
        builder.Connect(simple_car.get_output_port(2), ports[1])
        builder.Connect(aggregator.get_output_port(0), idm.get_input_port(2))
        builder.ExportOutput(aggregator.get_output_port(0))

        builder.ExportInput(pursuit.get_input_port(0))

        diagram = builder.Build()

        # Fix the lane input for now.
        context = diagram.CreateDefaultContext()
        value = framework.AbstractValue.Make(LaneDirection(lane_0, True))
        context.FixInputPort(0, value)

        # Set up the simulator.
        simulator = Simulator(diagram, context)
        simulator.Initialize()

        state = simulator.get_mutable_context().get_mutable_state() \
                         .get_mutable_continuous_state().get_mutable_vector()
        print(state.CopyToVector())

        # initial_state = np.array([0., 0., 0., 0.])
        # state.SetFromVector(initial_state)

        # Run a simulation step. This can go into a loop so that we can
        # increment the simulator, access the rollout, or modify the context
        # directly.
        simulator.StepTo(1.)

        print(state.CopyToVector())


if __name__ == '__main__':
    unittest.main()
