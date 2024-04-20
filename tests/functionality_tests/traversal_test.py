import math
import unittest
from engine.robot_logic.traversal import (
    ackerman_calculations,
    ackerman_right,
    ackerman_left,
)


class TestAckermanFunctions(unittest.TestCase):
    def test_ackerman_right(self):
        # Test case where the robot needs to turn right
        left_ang = math.radians(30)  # in radians
        right_ang = math.radians(45)  # in radians
        inside_ang = math.radians(30)  # in radians
        wheel_base = 1.5  # in meters
        steering_ratio = 1  # arbitrary value
        expected_turn_angle = math.radians(45 - inside_ang)
        actual_turn_angle = math.radians(
            ackerman_right(left_ang, right_ang, inside_ang, wheel_base, steering_ratio)
        )
        self.assertEqual(expected_turn_angle, actual_turn_angle)

    def test_ackerman_left(self):
        # Test case where the robot needs to turn left
        left_ang = math.radians(45)  # in radians
        right_ang = math.radians(30)  # in radians
        inside_ang = math.radians(45)  # in radians
        wheel_base = 1.5  # in meters
        steering_ratio = 1  # arbitrary value
        expected_turn_angle = math.radians(45 - inside_ang)
        actual_turn_angle = math.radians(
            ackerman_left(left_ang, right_ang, inside_ang, wheel_base, steering_ratio)
        )
        self.assertEqual(expected_turn_angle, actual_turn_angle)


if __name__ == "__main__":
    unittest.main()
