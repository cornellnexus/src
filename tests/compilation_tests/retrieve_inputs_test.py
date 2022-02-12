import unittest
from gui.retrieve_inputs import *

'''
Data parsing test for GUI's Input Validation
'''

class RetrieveInputTest(unittest.TestCase):
    def test_get_mode_frequency(self):
        phases = [0, 1, 2, 3, 4, 0]
        phase = get_mode(phases)
        self.assertEqual('0', phase)

    def test_get_phase_bounds(self): 
        mode = [4, 3, 20, 20, 20, 20, 20, 20, 20, 1]
        phase = get_mode(mode)
        self.assertEqual('4', phase)

    def test_get_median(self):
        data = [0.01, 0.22, 9.17, 5.00, 4.19, 3.55, 6.82]
        median = get_median(data)
        self.assertEqual('4.19', median)


if __name__ == '__main__':
    unittest.main()