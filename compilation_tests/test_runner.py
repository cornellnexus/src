import unittest
import node_test
import sim_traj_test

'''
Runs tests files
'''

loader = unittest.TestLoader()
suite  = unittest.TestSuite()

suite.addTests(loader.loadTestsFromModule(node_test))
suite.addTests(loader.loadTestsFromModule(sim_traj_test))
runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)