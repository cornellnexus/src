import unittest
from engine.packet import *

"""
Data parsing test for Packet Class
"""


class PacketTest(unittest.TestCase):
    def test_fix_data_size(self):
        init_data = "3.14"
        desired_int_len = 3
        desired_decimal_len = 5
        fixed_size = fix_data_size(
            init_data, desired_int_len, desired_decimal_len)
        self.assertEqual("003.14000", fixed_size)

        self.assertEqual("010.0", fix_data_size("10", 3, 1))

    def test_fix_values_size(self):
        self.assertEqual("010.0,009.0", fix_values_size(("10", "9"), 3, 1))

        self.assertEqual("009.74,031.59", fix_values_size(
            ("9.747", "31.592"), 3, 2))

        self.assertEqual("003.86,025.37", fix_values_size(
            ("3.865", "25.37"), 3, 2))

    def test_packet_str(self):
        test_packet = Packet(
            "1",
            "2.0",
            ["3.0", "0.0", "0.0"],
            "4.0",
            "5.",
            ["6.0", "7.0"],
            "8.0",
            ["9.0", "10.0"],
            ["11.0", "12.0", "0.1"],
            "13.0",
            "5",
        )
        correct_packet = "phase:1;p_weight:02.0;acc:3.00,0.00,0.00;n_dist:04.0;rot:05.00;last_n:006.00,007.00;vel:8.00;next_n:009.00,010.00;coord:011.00,012.00,000.10;batt:013;ctrl:5"
        self.assertEqual(str(test_packet), correct_packet)

        test_packet = Packet(
            "0",
            "0.9",
            ["3.2", "0.0", "0.0"],
            "1.7",
            "30.6",
            ["20.1", "30.2"],
            "5.0",
            ["20.2", "30.2"],
            ["20.15", "30.2", "1.0"],
            "50",
            "4",
        )
        correct_packet = (
            "phase:0;p_weight:00.9;acc:3.20,0.00,0.00;n_dist:01.7;rot:30.60;last_n:020.10,030.20;vel:5.00;"
            + "next_n:020.20,030.20;coord:020.15,030.20,001.00;batt:050;ctrl:4"
        )

        self.assertEqual(str(test_packet), correct_packet)


if __name__ == "__main__":
    unittest.main()
