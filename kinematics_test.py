from kinematics import *
import unittest

'''
Unit tests for kinematics.py
'''

class TestRobotGlobalConversions(unittest.TestCase):
  def test_robot_to_global(self):
    print("not used, what is global")
  def test_global_to_robot(self):
    print("not used, what is global")

class TestAlegbra(unittest.TestCase):
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
