import unittest
# import compilation_tests.astar_test as astar_test
import compilation_tests.database_test as database_test
import compilation_tests.gps_test as gps_test
import compilation_tests.grid_test as grid_test
import compilation_tests.imu_test as imu_test
import compilation_tests.kinematics_test as kinematics_test # Fixed
import compilation_tests.mag_heading_test as mag_heading_test 
import compilation_tests.movement_test as movement_test
import compilation_tests.node_test as node_test #TODO: FIX NODE TEST
import compilation_tests.packet_test as packet_test
import compilation_tests.pid_controller_test as pid_controller_test #fixed
import compilation_tests.retrieve_inputs_test as retrieve_inputs_test
import compilation_tests.robot_test as robot_test #TODO: FIX ROBOT_TEST
import compilation_tests.traverse_standard_test as traverse_standard_test # Fixed

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
compilation_tests = [
    database_test, gps_test,grid_test, imu_test, 
    kinematics_test, mag_heading_test, movement_test, 
    node_test,packet_test, 
    pid_controller_test, retrieve_inputs_test, robot_test,
    traverse_standard_test
    ]

for test_case in compilation_tests:
    suite.addTests(loader.loadTestsFromModule(test_case))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)
