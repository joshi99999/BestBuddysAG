#!/usr/bin/env python3

import unittest
import rclpy
import numpy as np
from rclpy.node import Node
from pos_control import PositionController

class PositionControllerTest(unittest.TestCase):

    def setUp(self):
        """self.desired_pos = np.array([0.01, 0.01, 0.01], dtype=np.float64)
        self.current_pos = np.array([0.01, 0.01, 0.01], dtype=np.float64)
        self.k = np.array([3, 3, 3], dtype=np.float64)
        self.vel_x_offset = 0.0
        self.grip = False
        self.max_vel = np.array([0.03, 0.04, 0.05], dtype=np.float64)
        self.max_acc = np.array([0.005, 0.005, 0.02], dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)"""
        self.controller = PositionController()

    def test_controller_behavior(self):
       # h = self.desired_pos, self.current_pos, self.k, self.vel_x_offset, self.grip, self.max_vel, self.max_acc, self.velocity
        #node = Node("position_controller_test_node")
        #controller1 = PositionController.calculate_control_signal(self)#, self.desired_pos, self.current_pos, self.vel_x_offset, self.grip)

        self.controller.current_pos = [0.0, 0.0, 0.0]
        self.controller.desired_pos = [0.0, 0.0, 0.0]

        result = self.controller.calculate_control_signal()
        self.assertEqual(result, 0)

        """test1 = PositionController.calculate_control_signal(controller1)
        self.assertEqual(test1.activate_gripper, controller1.grip)
        self.assertAlmostEqual(test1.vel_x, 0.0)  # Assuming Kp = 3.0 and initial velocity is 0.0
        self.assertAlmostEqual(test1.vel_y, 0.0)  # The desired position in Y-axis is 2.0, and the current position is 0.0
        self.assertAlmostEqual(test1.vel_z, 0.0)  # The desired position in Z-axis is 3.0, and the current position is 0.0"""

if __name__ == '__main__':
    unittest.main()




'''
from  pos_control import PositionController
import unittest
import numpy as np

class TestPosControl(unittest.TestCase):

    def setUp(self):
        self.k = np.array([3, 3, 3], dtype=np.float64)
        self.max_acc = np.array([0.005, 0.005, 0.02], dtype=np.float64)
        self.max_vel = np.array([0.03, 0.04, 0.05], dtype=np.float64)
        self.grip = False
        self.vel_x_offset = 0.01
        self.velocity = np.zeros(3, dtype=np.float64)
        self.desired_pos = [0.0, 0.0, 0.0]
        self.current_pos = [0.0, 0.0, 0.0]
        self.pos1 = PositionController(self.current_pos)


    def test_calculate_control_signal(self):
        
        pos1 = PositionController()

        self.assertEqual(pos1.calculate_control_signal, [0.0, 0.0, 0.0, False])


if __name__ == '__main__':
    unittest.main()

class TestPosControl(unittest.TestCase):

    def test_classCallback(self):
        current_pos = np.array([0.2, 0.2, 0,2], dtype=np.float64)
        desired_pos = np.array([0.1, 0.1, 0.1], dtype=np.float64)
        pos_control.calculate_control_signal(current_pos, desired_pos)
        vel_x_offset = 0.1
        assert calculate_control_signal(current_pos, desired_pos, vel_x_offset) == np.array([0.1, 0.1, 0.1], dtype=np.float64)
if __name__ == '__main__':
    unittest.main()

'''
