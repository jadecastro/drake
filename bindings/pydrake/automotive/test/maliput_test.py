#!/usr/bin/env python

from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.automotive.maliput import (
    RoadGeometry,
    RoadGeometryId,
    )
from pydrake.systems.analysis import (
    Simulator
    )
from pydrake.systems.primitives import (
    ConstantVectorSource,
    )


class TestPendulum(unittest.TestCase):
    def test_maliput(self):
        rg_id = RoadGeometryId("ace")
        rg = RoadGeometry(rg_id, 1, 100., 4., 0., 1., 1e-6, 1e-6);
        print(rg.linear_tolerance())

if __name__ == '__main__':
    unittest.main()
