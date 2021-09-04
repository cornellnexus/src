import unittest
import compilation_tests.node_test as node_test
import compilation_tests.sim_traj_test as sim_traj_test
import compilation_tests.pid_controller_test as pid_controller_test
import compilation_tests.state_manager_test as state_manager_test
import compilation_tests.robot_test as robot_test
import compilation_tests.kinematics_test as kinematics_test

'''
Runs all compilation test files as individual modules
'''

loader = unittest.TestLoader()
suite = unittest.TestSuite()

# adding tests:
compilation_tests = [node_test, sim_traj_test, pid_controller_test, state_manager_test, robot_test, kinematics_test]
for test_case in compilation_tests:
    suite.addTests(loader.loadTestsFromModule(test_case))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)
