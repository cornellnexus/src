import unittest

import tests.compilation_tests.pid_controller_test as pid_controller_test #fixed
import tests.compilation_tests.kinematics_test as kinematics_test # Fixed
import tests.compilation_tests.robot_test.traverse_standard_test as traverse_standard_test # Fixed
import tests.compilation_tests.robot_test.execute_setup_test as execute_setup_test #fixed 
import tests.compilation_tests.robot_test.robot_test as robot_test #fixed 
import tests.compilation_tests.packet_test as packet_test #fixed
import tests.compilation_tests.retrieve_inputs_test as retrieve_inputs_test #fixed
import tests.compilation_tests.mag_heading_test as mag_heading_test #fixed
import tests.compilation_tests.grid_test as grid_test #fixed
import tests.compilation_tests.node_test as node_test #fixed
import compilation_tests.database_test as database_test #TODO




'''
Runs all compilation test files as individual modules
'''

loader = unittest.TestLoader()
suite = unittest.TestSuite()

# adding tests:
compilation_tests = [pid_controller_test, kinematics_test, traverse_standard_test, execute_setup_test, 
robot_test, packet_test, retrieve_inputs_test, mag_heading_test, grid_test, node_test]

for test_case in compilation_tests:
    suite.addTests(loader.loadTestsFromModule(test_case))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)
