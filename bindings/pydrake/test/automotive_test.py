#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.automotive import (
    DrivingCommand,
    LaneDirection,
    IdmController,
    PurePursuitController,
    RoadPositionStrategy,
    ScanStrategy,
    SimpleCar,
    create_lane_direction,
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
    PassThrough,
    )
from pydrake.systems.rendering import (
    create_frame_velocity,
    FrameVelocity,
    PoseAggregator,
    PoseVector,
    )


# Construct a IDM-controlled SimpleCar in a dragway, a middle-ground between IDM
# and MOBIL.  Essentially it's a redeaux of
# AutomotiveSimulator::IdmControlledCar.
class TestAutomotiveDiagram(unittest.TestCase):
    def test_pure_pursuit(self):
        print("PurePursuit")
        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("two-lane road")
        rg = create_dragway(rg_id, 2, 100., 4., 0., 1., 1e-6, 1e-6)
        segment = rg.junction(0).segment(0)
        lane_0 = rg.junction(0).segment(0).lane(0)

        pure_pursuit = PurePursuitController()
        context = pure_pursuit.CreateDefaultContext()
        output = pure_pursuit.AllocateOutput(context)

        # Fix the lane direction.
        ld_value = framework.AbstractValue.Make(LaneDirection(lane_0, True))
        context.FixInputPort(0, ld_value)

        # Fix the pose.
        pos = [0., 0., 0.]
        pose_vector = PoseVector()
        pose_vector.set_translation(pos)
        context.FixInputPort(1, pose_vector)

        # Check the correctness of the inputs.
        pose_vector_eval = pure_pursuit.EvalVectorInput(context, 1)
        print(pose_vector_eval.get_value())

        pure_pursuit.CalcOutput(context, output)
        output_value = output.get_vector_data(0)
        print(output_value.get_value())

    def test_idm_controller(self):
        print("IDM")
        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("two-lane road")
        rg = create_dragway(rg_id, 2, 100., 4., 0., 1., 1e-6, 1e-6)

        builder = framework.DiagramBuilder()

        idm = builder.AddSystem(IdmController(
            rg, ScanStrategy.kPath, RoadPositionStrategy.kExhaustiveSearch, 0.))
        aggregator = builder.AddSystem(PoseAggregator())
        ports = aggregator.AddSinglePoseAndVelocityInput("idm", 0)

        # TODO: Replace magic numbers with named getters.
        builder.Connect(aggregator.get_output_port(0), idm.get_input_port(2))

        builder.ExportOutput(idm.get_output_port(0))
        pose1_index = builder.ExportInput(idm.get_input_port(0))
        velocity1_index = builder.ExportInput(idm.get_input_port(1))
        pose2_index = builder.ExportInput(ports[0])
        velocity2_index = builder.ExportInput(ports[1])

        diagram = builder.Build()

        context = diagram.CreateDefaultContext()
        output = diagram.AllocateOutput(context)

        # Fix the pose.
        pos = [0., 0., 0.]
        pose_vector1 = PoseVector()
        pose_vector1.set_translation(pos)
        context.FixInputPort(pose1_index, pose_vector1)
        pose_vector2 = PoseVector()
        pose_vector2.set_translation(pos)
        context.FixInputPort(pose2_index, pose_vector2)

        # Fix the frame velocity.
        w = [0., 0., 0.]
        v = [1., 0., 0.]
        frame_velocity1 = create_frame_velocity(w, v)
        context.FixInputPort(velocity1_index, frame_velocity1)
        frame_velocity2 = create_frame_velocity(w, v)
        context.FixInputPort(velocity2_index, frame_velocity2)

        # Check the correctness of the inputs.
        pose_vector_eval = diagram.EvalVectorInput(context, pose1_index)
        print(pose_vector_eval.get_value())
        frame_velocity_eval = diagram.EvalVectorInput(context, velocity1_index)
        print(frame_velocity_eval.get_value())

        diagram.CalcOutput(context, output)
        output_value = output.get_vector_data(0)
        print(output_value.get_value())

    def test_simple_car(self):
        print("Simple Car")
        simple_car = SimpleCar()

        simulator = Simulator(simple_car)
        context = simulator.get_mutable_context()
        output = simple_car.AllocateOutput(context)

        command = DrivingCommand()
        command.set_steering_angle(0.5)
        command.set_acceleration(1.)
        context.FixInputPort(0, command)

        # Check the correctness of the inputs.
        command_eval = simple_car.EvalVectorInput(context, 0)
        print(command_eval.get_value())

        state = context.get_mutable_continuous_state_vector()
        print(state.CopyToVector())

        simulator.StepTo(1.0)

        simple_car.CalcOutput(context, output)
        output_value = output.get_vector_data(0)
        print(state.CopyToVector())
        print(output_value.get_value())

    def test_driving_command(self):
        # TODO: Fill me in.
        pass

    def test_idm_dragway(self):
        print("Dragway diagram")
        # Instantiate a two-lane straight road.
        rg_id = RoadGeometryId("two-lane road")
        rg = create_dragway(rg_id, 2, 100., 4., 0., 1., 1e-6, 1e-6)
        segment = rg.junction(0).segment(0)
        lane_0 = segment.lane(0)
        lane_1 = segment.lane(1)

        # Build a diagram with the IDM car placed in lane 0.
        builder = framework.DiagramBuilder()

        # TODO: used named variables
        idm = builder.AddSystem(IdmController(
            rg, ScanStrategy.kPath, RoadPositionStrategy.kExhaustiveSearch, 0.))
        simple_car = builder.AddSystem(SimpleCar())
        pursuit = builder.AddSystem(PurePursuitController())
        mux = builder.AddSystem(Multiplexer(DrivingCommand()))
        aggregator = builder.AddSystem(PoseAggregator())
        ports = aggregator.AddSinglePoseAndVelocityInput("idm_car_0", 0)

        # TODO(jadecastro) Use named port getters provided by each system.
        builder.Connect(simple_car.get_output_port(1), idm.get_input_port(0))
        builder.Connect(simple_car.get_output_port(2), idm.get_input_port(1))
        builder.Connect(simple_car.get_output_port(1), pursuit.get_input_port(1))
        builder.Connect(pursuit.get_output_port(0), mux.get_input_port(0))
        builder.Connect(idm.get_output_port(0), mux.get_input_port(1))
        builder.Connect(mux.get_output_port(0), simple_car.get_input_port(0))
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

        state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
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
