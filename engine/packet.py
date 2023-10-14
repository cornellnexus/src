"""
Packet functionality
"""


def fix_data_size(s, desired_int_length, desired_decimal_length):
    """
    Args:
        s: input string representing a numeric value
        desired_int_length: integer representing the number of digits that should appear before the decimal point
        desired_decimal_length: integer representing the number of digits that should appear after the decimal point

    Returns: string of [s] numeric value with the correct size/length. For example, fix_data_size("10", 3, 1) should output "010.0", which
    is of the same value as s. Note that there are 3 digits before the decimal point (desire_int_length) and
    1 digit after the decimal point (desired_decimal_length).

    """

    separator_index = s.find(".")
    has_deci = True
    if separator_index == -1:
        has_deci = False
        separator_index = len(s)
    current_int_len = len(s[0:separator_index])
    current_deci_len = len(s[separator_index + 1 :])
    int_difference = desired_int_length - current_int_len
    deci_difference = desired_decimal_length - current_deci_len

    if int_difference < 0:
        # shrink:
        print("value of " + s + " too large?? possibly round?")
    else:
        # extend: add 0's to the beginning to extend integer value
        int_buffer = "0" * int_difference
        s = int_buffer + s

    if not has_deci:
        s = s + "."
    if deci_difference < 0:
        # shrink: cut off extra decimal values
        s = s[0 : len(s) + deci_difference]
    else:
        # extend: add 0's to the end to extend decimal values
        deci_buffer = "0" * deci_difference
        s = s + deci_buffer

    return s


def fix_values_size(str_vals, desired_int_length, desired_decimal_length):
    """

    Args:
        str_vals: list of strings representing numerical values
        desired_int_length: integer representing the number of digits that should appear before the decimal point
        desired_decimal_length: integer representing the number of digits that should appear after the decimal point

    Returns: string representing a tuple of same numeric values as [t] with the correct size/length for each entry.
    For example, fix_values_size(["10","9"], 3, 1) should output "010.0,009.0", which contains of the same numeric
    values as [t]. Note that there are 3 digits before the decimal point (desire_int_length) and 1 digit after the
    decimal point (desired_decimal_length) for the str_val values.

    """
    formatted_str = ""
    for i in range(len(str_vals)):
        s = str_vals[i]
        if i == len(str_vals) - 1:
            formatted_str += fix_data_size(
                s, desired_int_length, desired_decimal_length
            )
        else:
            formatted_str += (
                fix_data_size(s, desired_int_length, desired_decimal_length) + ","
            )

    return formatted_str


class Packet:
    """
    Args:
        phase: string representing phase value, must be a single digit such as "0"
        p_weight: string representing weight value
        acc: string representing acceleration value
        n_dist: tuple of strings representing the next node distance
        rot: string representing rotation value
        last_n: tuple of strings representing the coordinates of the last node visited
        vel: string representing velocity value
        next_n: tuple of strings representing the coordinates of the next node to visit
        coord: tuple of strings representing the coordinates and heading of the robot. Ex: ("6.0", "7.0", "1.0")
        batt: string representing the remaining battery percentage
        ctrl: string representing control mode value, must be a single digit such as "1"

    Returns: string of the following format using correcting size of args
    "phase:0;p_weight:00.0;acc:0.00,0.00,0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00,000.00;batt:000;ctrl:1"
    """

    def __init__(
        self, phase, p_weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl
    ):
        self.phase = phase
        self.p_weight = p_weight
        self.acc = acc
        self.n_dist = n_dist
        self.rot = rot
        self.last_n = last_n
        self.vel = vel
        self.next_n = next_n
        self.coord = coord
        self.batt = batt
        self.ctrl = ctrl

    def __str__(self):
        """
        Returns: string of the following format using correcting size of args
        "phase:0;p_weight:00.0;acc:0.00,0.00,0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coord:000.00,000.00;batt:000;ctrl:1"
        """
        args = [
            ("phase:", self.phase),
            (";p_weight:", fix_data_size(self.p_weight, 2, 1)),
            (";acc:", fix_values_size(self.acc, 1, 2)),
            (";n_dist:", fix_data_size(self.n_dist, 2, 1)),
            (";rot:", fix_data_size(self.rot, 2, 2)),
            (";last_n:", fix_values_size(self.last_n, 3, 2)),
            (";vel:", fix_data_size(self.vel, 1, 2)),
            (";next_n:", fix_values_size(self.next_n, 3, 2)),
            (";coord:", fix_values_size(self.coord, 3, 2)),
            (";batt:", fix_data_size(self.batt + ".0", 3, -1)),
            (";ctrl:", self.ctrl),
        ]

        packet = ""
        for (heading, value) in args:
            packet += heading + str(value)

        return packet
