import unittest

'''
Unit tests for kinematics.py

TODO: Write test cases
'''


class TestKinematics(unittest.TestCase):
    def test_robot_global_conversions(self):
        pass

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
