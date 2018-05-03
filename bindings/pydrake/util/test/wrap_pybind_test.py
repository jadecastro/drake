from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.util.wrap_test_util import (
    MyContainer,
    MyValue,
)


class TestWrapPybind(unittest.TestCase):
    def test_read_write_keep_alive(self):
        # Test the containers' constructors and accessors.
        my_value = MyValue()
        my_value.value = 42.
        my_container = MyContainer()
        my_container.member = my_value
        print("my_container's value: " + str(my_container.get_value()))
        print("my_value's value: " + str(my_value.value))
        my_container.member = None
        print("my_container's member: " + str(my_container.member))
        print("my_container's new value: " + str(my_container.member.value))
        self.assertEqual(my_container.get_value(), 42.)
