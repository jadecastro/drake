#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.automotive import (
    SimpleCar,
    create_driving_command,
    )
from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.primitives import (
    ConstantVectorSource,
    )


class TestSimpleCar(unittest.TestCase):
    def test_simulation(self):
        simple_car = SimpleCar()
        command = create_driving_command()
        value = command.get_mutable_value()
        # TODO(jadecastro) Implement with named vectors.
        value[0] = 1.
        value[1] = 1.
        print(command.get_value())

        context = simple_car.CreateDefaultContext()
        context.FixInputPort(0, command)

        simulator = Simulator(simple_car, context)
        simulator.Initialize()

        state = simulator.get_mutable_context().get_mutable_state() \
                         .get_mutable_continuous_state().get_mutable_vector()

        # TODO(jadecastro) Used named vectors.
        initial_state = np.array([0., 0., 0., 0.])
        state.SetFromVector(initial_state)

        simulator.StepTo(1.)

        print(state.CopyToVector())


if __name__ == '__main__':
    unittest.main()
