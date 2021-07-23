import unittest
import numpy as np
import math
from engine.kinematics import robot_to_global, global_to_robot

'''
Unit tests for kinematics.py
'''

class TestKinematics(unittest.TestCase):
  def test_robot_global_conversions(self):
    self.assertEqual(np.array([[-4],[-2]]).tolist(), robot_to_global([1,4,math.pi], 5, 6).tolist())
    self.assertEqual(np.array([[-5],[-2]]).tolist(), global_to_robot([1,4,math.pi], 5, 6).tolist())

# There should be a bunch of tests in each of the functions below
  def test_feedback_lin(self):
    pass

  def test_limit_cmds(self):
    pass

  def test_integrate_odom(self):
    pass

  def test_meters_to_gps(self):
    pass

  def test_meters_to_lat(self):
    pass

  def test_meters_to_long(self):
    pass

# Should now include the tests for test_get_vincenty_x and test_get_vincenty_y :
  def test_vincenty(self):
    pass
# Should now include the tests for test_get_haversine_x and test_get_haversine_y :
  def test_haversine(self):
    pass


if __name__ == '__main__':
    unittest.main()
