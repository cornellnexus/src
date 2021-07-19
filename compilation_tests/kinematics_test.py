import os.path
cwd = os.getcwd()
import sys
sys.path.append(cwd[0:cwd.index('compilation_tests')-1]+"/software")

from kinematics import *
import unittest
import math

'''
Unit tests for kinematics.py
'''

class TestRobotGlobalConversions(unittest.TestCase):
  def test_robot_to_global(self):
    self.assertEqual(np.array([[-4],[-2]]).tolist(), robot_to_global([1,4,math.pi], 5, 6).tolist())
  
  def test_global_to_robot(self):
    self.assertEqual(np.array([[-5],[-2]]).tolist(), global_to_robot([1,4,math.pi], 5, 6).tolist())

class TestAlgebra(unittest.TestCase):
  def test_feedback_lin(self):
    print("test")
  def test_limit_cmds(self):
    print("test")
  def test_integrate_odom(self):
    print("test")

class TestMeterConversions(unittest.TestCase):
  def test_meters_to_gps(self):
    print("test")
  def test_meters_to_lat(self):
    print("test")
    # self.assertEqual(0.1, meters_to_lat(185)) 
  def test_meters_to_long(self):
    print("test")

class TestVincentyConversions(unittest.TestCase):
  def test_get_vincenty_x(self):
    print("test")
  def test_get_vincenty_y(self):
    print("test")
  def test_get_haversine_x(self):
    print("test")
  def test_get_haversine_y(self):
    print("test")


# class TestHaversineConversions(unittest.TestCase):



if __name__ == '__main__':
    unittest.main()
