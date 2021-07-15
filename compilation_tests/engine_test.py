import os.path
cwd = os.getcwd()
import sys
sys.path.append(cwd[0:cwd.index('compilation_tests')-1])
from sim_trajectory import *
import unittest

class Test(unittest.TestCase):
  def test(self):
    print("hello")



if __name__ == '__main__':
    unittest.main()
