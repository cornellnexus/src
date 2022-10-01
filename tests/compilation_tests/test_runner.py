import unittest
import tests.compilation_tests.pid_controller_test as pid_controller_test 
import tests.compilation_tests.kinematics_test as kinematics_test 
import tests.compilation_tests.robot_test.traverse_standard_test as traverse_standard_test 
import tests.compilation_tests.robot_test.execute_setup_test as execute_setup_test  
import tests.compilation_tests.robot_test.robot_test as robot_test  
import tests.compilation_tests.packet_test as packet_test 
import tests.compilation_tests.retrieve_inputs_test as retrieve_inputs_test 
import tests.compilation_tests.mag_heading_test as mag_heading_test 
import tests.compilation_tests.grid_test as grid_test 
import tests.compilation_tests.node_test as node_test 
import tests.compilation_tests.database_test as database_test 

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
compilation_tests = [pid_controller_test, kinematics_test, 
    traverse_standard_test,execute_setup_test,robot_test,packet_test,
    retrieve_inputs_test, mag_heading_test,grid_test, node_test, database_test]
for test_case in compilation_tests:
    suite.addTests(loader.loadTestsFromModule(test_case))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)
