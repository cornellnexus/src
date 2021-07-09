import os.path
cwd = os.getcwd()
import sys
sys.path.append(cwd[0:cwd.index('compilation_tests')-1])
from engine_rpi import *


class Test(unittest.TestCase):
  def test(self):




if __name__ == '__main__':
    unittest.main()
