import unittest
# import compilation_tests.node_test as node_test //TODO: UPDATE NODE_TEST
import tests.compilation_tests.pid_controller_test as pid_controller_test
# import compilation_tests.robot_test as robot_test //TODO: FIX ROBOT_TEST
import tests.compilation_tests.kinematics_test as kinematics_test
import tests.compilation_tests.traverse_standard_test as traversal_standard_test
# import compilation_tests.grid_test as grid_test //TODO: UPDATE NODE_TEST

'''
Runs all compilation test files as individual modules

Compilation test files DO NOT INCLUDE: 
-gps_test
-imu_test
-mag_heading_test
-movement_test
'''

loader = unittest.TestLoader()
suite = unittest.TestSuite()

# adding tests:
compilation_tests = [pid_controller_test, kinematics_test, traversal_standard_test]
for test_case in compilation_tests:
    suite.addTests(loader.loadTestsFromModule(test_case))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)
