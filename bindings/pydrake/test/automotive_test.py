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
    SimpleCar,
    create_lane_direction,
    create_driving_command,
    )
from pydrake.maliput.api import (
    RoadGeometryId
    )
from pydrake.maliput.dragway import (
    RoadGeometry
    )
from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.primitives import (
    Multiplexer,
    )
from pydrake.systems.rendering import (
    PoseAggregator,
    )


# Construct a IDM-controlled SimpleCar in a dragway, a middle-ground between IDM
# and MOBIL.  Essentially it's a redeaux of
# AutomotiveSimulator::IdmControlledCar.
class TestAutomotiveDiagram(unittest.TestCase):
    def test_idm_dragway(self):
        rg_id = RoadGeometryId("ace")
        rg = RoadGeometry(rg_id, 1, 100., 4., 0., 1., 1e-6, 1e-6);

        junction = rg.junction(0)
        segment = junction.segment(0)
        lane = segment.lane(0)

        builder = framework.DiagramBuilder()

        idm = builder.AddSystem(IdmController(rg, RoadPositionStrategy.kExhaustiveSearch, 0.))
        simple_car = builder.AddSystem(SimpleCar())
        pursuit = builder.AddSystem(PurePursuitController())
        mux = builder.AddSystem(Multiplexer(create_driving_command()))
        aggregator = builder.AddSystem(PoseAggregator())

        # TODO(jadecastro) Use convenience getters provided by each system.
        builder.Connect(simple_car.get_output_port(1), idm.get_input_port(0))
        builder.Connect(simple_car.get_output_port(2), idm.get_input_port(1))
        builder.Connect(pursuit.get_output_port(0), mux.get_input_port(0))
        builder.Connect(idm.get_output_port(0), mux.get_input_port(1));
        builder.Connect(mux.get_output_port(0), simple_car.get_input_port(0))

        ports = aggregator.AddSinglePoseAndVelocityInput("my_first_car", 0)

        builder.Connect(simple_car.get_output_port(1), ports[0]);
        builder.Connect(simple_car.get_output_port(2), ports[1]);
        builder.Connect(aggregator.get_output_port(0), idm.get_input_port(2))
        builder.ExportOutput(aggregator.get_output_port(0));

        builder.ExportInput(pursuit.get_input_port(0))

        diagram = builder.Build()

        context = diagram.CreateDefaultContext()
        value = framework.AbstractValue.Make(LaneDirection(lane, True))
        context.FixInputPort(0, value)

        simulator = Simulator(diagram, context)
        simulator.Initialize()

        state = simulator.get_mutable_context().get_mutable_state() \
                         .get_mutable_continuous_state().get_mutable_vector()
        print(state.CopyToVector())

        # initial_state = np.array([0., 0., 0., 0.])
        # state.SetFromVector(initial_state)

        simulator.StepTo(1.)

        print(state.CopyToVector())


if __name__ == '__main__':
    unittest.main()
