import unittest
from gui.validate_inputs import *

'''
Data parsing test for GUI's Input Validation
'''

class ValidateInputTest(unittest.TestCase):
    def test_get_phase_frequency(self):
        phases = [0, 1, 2, 3, 4, 0]
        phase = get_phase(phases)
        self.assertEqual('0', phase)

    def test_get_phase_bounds(self): 
        phases = [4, 3, 5, 5, 5, 5, 5, 5, 5, 1]
        phase = get_phase(phases)
        self.assertEqual('4', phase)

    def test_get_median(self):
        data = [0.01, 0.22, 9.17, 5.00, 4.19, 3.55, 6.82]
        median = get_median(data)
        self.assertEqual('4.19', median)

    def test_fix_data_size(self): 
        init_data = '3.14'
        desired_int_len = 3
        desired_decimal_len = 5
        fixed_size =  fix_data_size(init_data, desired_int_len, desired_decimal_len)
        self.assertEqual('003.14000', fixed_size)

    def test_build_validated_packet(self):
        test_packet = build_validated_packet(0, 0.9, 3.2, 1.7, 30.6, (20.1, 30.2), 5.0, (20.2, 30.2), (20.15, 30.2), 50.6)
        correct_packet = "phse:0;p_weight:00.9;acc:3.20;n_dist:01.7;rot:30.60;last_n:(020.10, 030.20);vel:5.00;" + \
        "next_n:(020.20, 030.20);coords:(020.15, 030.2);bat:505"
        self.assertEqual(test_packet, correct_packet)

if __name__ == '__main__':
    unittest.main()