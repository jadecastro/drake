#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import unittest
import numpy as np

from pydrake.systems import (
    # framework
    BasicVector,
    Diagram,
    DiagramBuilder,
    # WitnessFunctionDirection
    # primitives
    Adder,
    ConstantVectorSource,
    Integrator,
    # analysis
    Simulator,
    )


class TestSystemGeneral(unittest.TestCase):
    def test_simulator_ctor(self):
        # Create simple system.
        system = ConstantVectorSource([1])

        # Create simulator with basic constructor.
        simulator = Simulator(system)
        simulator.Initialize()
        simulator.StepTo(1)

        # Create simulator specifying context.
        context = system.CreateDefaultContext()
        simulator = Simulator(system, context)
        simulator.StepTo(1)

    def test_basic_diagram(self):
        # Similar to: //systems/framework:diagram_test, ExampleDiagram
        size = 3

        builder = DiagramBuilder()
        adder0 = builder.AddSystem(Adder(2, size))
        adder0.set_name("adder0")
        adder1 = builder.AddSystem(Adder(2, size))
        adder1.set_name("adder1")

        integrator = builder.AddSystem(Integrator(size))
        integrator.set_name("integrator")

        builder.Connect(adder0.get_output_port(0), adder1.get_input_port(0))
        builder.Connect(adder1.get_output_port(0),
                        integrator.get_input_port(0))

        builder.ExportInput(adder0.get_input_port(0))
        builder.ExportInput(adder0.get_input_port(1))
        builder.ExportInput(adder1.get_input_port(1))
        builder.ExportOutput(integrator.get_output_port(0))

        diagram = builder.Build()
        # TODO(eric.cousineau): Figure out simple unicode handling, if
        # necessary. (Restore the snowman.)
        diagram.set_name("test_diagram")

        simulator = Simulator(diagram)
        context = simulator.get_mutable_context()

        # Create and attach inputs.
        # TODO(eric.cousineau): Not seeing any assertions being printed if no
        # inputs are connected. Need to check this behavior.
        input0 = BasicVector(np.array([0.1, 0.2, 0.3]))
        context.FixInputPort(0, input0)
        input1 = BasicVector([0.02, 0.03, 0.04])
        context.FixInputPort(1, input1)
        input2 = BasicVector([0.003, 0.004, 0.005])
        context.FixInputPort(2, input2)

        # Initialize integrator states.
        def get_mutable_continuous_state(system):
            return (diagram.GetMutableSubsystemState(system, context)
                           .get_mutable_continuous_state())

        integrator_xc = get_mutable_continuous_state(integrator)
        integrator_xc.get_mutable_vector().SetFromVector([0, 1, 2])

        simulator.Initialize()

        # Simulate briefly, and take full-context snapshots at intermediate
        # points.
        n = 6
        times = np.linspace(0, 1, n)
        context_log = []
        for t in times:
            simulator.StepTo(t)
            # Record snapshot of *entire* context.
            context_log.append(context.Clone())

        xc_initial = np.array([0, 1, 2])
        xc_final = np.array([0.123, 1.234, 2.345])

        for i, context_i in enumerate(context_log):
            t = times[i]
            self.assertEqual(context_i.get_time(), t)
            xc = (context_i.get_state().get_continuous_state()
                           .get_vector().CopyToVector())
            xc_expected = (float(i) / (n - 1) * (xc_final - xc_initial) +
                           xc_initial)
            print("xc[t = {}] = {}".format(t, xc))
            self.assertTrue(np.allclose(xc, xc_expected))


if __name__ == '__main__':
    unittest.main()
