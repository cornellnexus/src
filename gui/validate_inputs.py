import os
import sys
import serial
import statistics

# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)


def update_gui():
    '''

    Reads telemetry data packets from raspberry pi, which ideally of the following format
    "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000;ctrl:1"

    From every 5 data packets, a single data packet is created to estimate the packets. This single data packet
    is written to robot_data.csv, so we can accurately display current information regarding the robot on the GUI.

    '''
    packets = []
    robot_data_file = open((get_path('csv')[-1] + '/robot_data.csv'), "a")  # open csv file of robot data

    while True:
        while len(packets) < 5:
            # packet = ser.readline()
            packet = input("Enter data: ") # use this for testing purposes
            if 80 < len(packet) < 150:  # check if packet length is appropriate
                packets.append(packet)
        valid_packet = validate_packet(packets)

        robot_data_file.write(valid_packet + '\n')
        packets = []

    robot_data_file.close()


def get_path(folder):
    '''
    Args:
        folder: a string of folder directory

    Returns: sys path to desired folder
    '''
    cwd = os.getcwd()
    sys.path.append(cwd + "/" + folder)
    return sys.path


def get_values(packet):
    '''
    Args:
        packet: a string of format "id:value"

    Returns: string of [data]'s numeric value (substring after :)
    '''
    separator_index = packet.find(":")
    if separator_index == -1:
        print("data corruption")
        raise Exception()
    return packet[separator_index + 1:]


def get_integer_value(packet):
    '''

    Args:
        packet: a string of format "id:value"

    Returns: integer of "value"

    '''
    return int(get_values(packet))


def get_float_value(packet):
    '''

    Args:
        packet: a string of format "id:value"

    Returns: float of "value"

    '''
    return float(get_values(packet))


def get_tuple_value(data):
    '''

    Args:
        data: a string of format "id:value"

    Returns: tuple of floats of "value"

    '''
    s = get_values(data)
    separator_index = s.find(",")
    if separator_index == -1:
        print("tuple data corruption")
        raise Exception()
    fst = s[:separator_index]
    snd = s[separator_index + 1:]
    return (float(fst), float(snd))


def validate_packet(packets):
    '''
    Args:
        packets: a list of strings containing telemetry data

    Returns: a single packet string of data representing median/average of valid packets in [packets].
    '''
    phases = []
    weights = []
    accs = []
    n_dists = []
    rots = []
    last_ns = []
    vels = []
    next_ns = []
    coords = []
    batts = []
    ctrls = []

    for packet in packets:
        packet_data = packet.split(";")
        phases.append(get_integer_value(packet_data[0]))
        weights.append(get_float_value(packet_data[1]))
        accs.append(get_float_value(packet_data[2]))
        n_dists.append(get_float_value(packet_data[3]))
        rots.append(get_float_value(packet_data[4]))
        last_ns.append(get_tuple_value(packet_data[5]))
        vels.append(get_float_value(packet_data[6]))
        next_ns.append(get_tuple_value(packet_data[7]))
        coords.append(get_tuple_value(packet_data[8]))
        batts.append(get_integer_value(packet_data[9]))
        ctrls.append(get_integer_value(packet_data[10]))

    phase = get_mode(phases)
    weight = get_median(weights)
    acc = get_median(accs)
    n_dist = get_median(n_dists)
    rot = get_median(rots)
    last_n = get_coord(last_ns)
    vel = get_median(vels)
    next_n = get_coord(next_ns)
    coord = get_coord(coords)
    batt = get_median(batts)
    ctrl = get_mode(ctrls)

    # return packet with combined data --> need to extend or shrink value to match data string
    return build_validated_packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl)


def build_validated_packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl):
    '''

    Args:
        phase: string representing phase value, must be a single digit such as "0"
        weight: string representing weight value
        acc: string representing acceleration value
        n_dist: tuple of strings representing the next node distance
        rot: string representing rotation value
        last_n: tuple of strings representing the coordinates of the last node visited
        vel: string representing velocity value
        next_n: tuple of strings representing the coordinates of the next node to visit
        coord: tuple of strings representing the coordinates of the robot. Ex: ("6.0", "7.0")
        batt: string representing the remaining battery percentage
        ctrl: string representing control mode value, must be a single digit such as “1"


    Returns: string of the following format using correcting size of args
    "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000;ctrl:1"
    '''
    return "phse:" + phase + ";p_weight:" + fix_data_size(weight, 2, 1) + ";acc:" + fix_data_size(acc, 1, 2) + ";n_dist:" + \
           fix_data_size(n_dist, 2, 1) + ";rot:" + fix_data_size(rot, 2, 2) + ";last_n:" + fix_tuple_size(last_n, 3, 2) + \
           ";vel:" + fix_data_size(vel, 1, 2) + ";next_n:" + fix_tuple_size(next_n, 3, 2) + \
           ";coords:" + fix_tuple_size(coord, 3, 2) + ";bat:" + fix_data_size(batt+".0", 3, -1) + ";ctrl:" + ctrl


def fix_data_size(s, desired_int_length, desired_decimal_length):
    '''

    Args:
        s: input string representing a numeric value
        desired_int_length: integer representing the number of digits that should appear before the decimal point
        desired_decimal_length: integer representing the number of digits that should appear after the decimal point

    Returns: string of [s] numeric value with the correct size/length. For example, fix_data_size("10", 3, 1) should output "010.0", which
    is of the same value as s. Note that there are 3 digits before the decimal point (desire_int_length) and
    1 digit after the decimal point (desired_decimal_length).

    '''

    separator_index = s.find(".")
    current_int_len = len(s[0:separator_index])
    current_deci_len = len(s[separator_index + 1:])
    int_difference = desired_int_length - current_int_len
    deci_difference = desired_decimal_length - current_deci_len

    if int_difference < 0:
        # shrink:
        print("value too large?? possibly round?")
    else:
        # extend: add 0's to the beginning to extend integer value
        int_buffer = "0" * int_difference
        s = int_buffer + s

    if deci_difference < 0:
        # shrink: cut off extra decimal values
        s = s[0:len(s) + deci_difference]
    else:
        # extend: add 0's to the end to extend decimal values
        deci_buffer = "0" * deci_difference
        s = s + deci_buffer

    return s


def fix_tuple_size(t, desired_int_length, desired_decimal_length, ):
    '''

    Args:
        t: input string representing a tuple of numeric values
        desired_int_length: integer representing the number of digits that should appear before the decimal point
        desired_decimal_length: integer representing the number of digits that should appear after the decimal point

    Returns: string representing a tuple of same numeric values as [t] with the correct size/length for each entry.
    For example, fix_data_size(("10","9"), 3, 1) should output ("010.0","009.0"), which contains of the same numeric
    values as [t]. Note that there are 3 digits before the decimal point (desire_int_length) and 1 digit after the
    decimal point (desired_decimal_length) for both the first and second tuple parameters.

    '''
    return fix_data_size(t[0], desired_int_length, desired_decimal_length) + "," +\
            fix_data_size(t[1], desired_int_length, desired_decimal_length)


def get_mode(data):
    '''

    Args:
        data: list of floats representing data values, such as phases or control modes

    Returns: a single string of the mode of [data] of format "0"

    '''
    processed_data = [x for x in data if x <= 7]
    final_data = statistics.mode(processed_data)
    return str(int(final_data))


def get_median(data_list):
    '''

    Args:
        data_list: list of floats

    Returns: a single string of the median of [data_list]

    '''
    return str(statistics.median(data_list))


def get_coord(coords):
    '''

    Args:
        coords: list of tuples of floats

    Returns: a tuple of strings where the first and second entries are the medians of [coords]

    '''
    x = []
    y = []
    for coord in coords:
        x.append(coord[0])
        y.append(coord[1])
    x_median = get_median(x)
    y_median = get_median(y)
    return (str(x_median), str(y_median))


# send 5 packets, then pause, then repeat/continue
# "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000"
### preset length: 131 characters

#### From raspberry pi
# Robot Phase: phse (mission)
# [SETUP: 0, AVOID_OBSTACLE: 1, RETURN: 2, DOCKING: 3, COMPLETE: 4]
# Pounds of Collected Plastic: p_weight
# Acceleration: acc (imu)
# Current Distance to Next Node: n_dist (mission)
# Rotation: rot (imu)
# Last Node Visited: last_n (mission)
# Velocity: vel (imu)
# Next Node to Visit: next_n (mission)
# Current Coordinates: coords
# Battery level: bat

## Calculated here
# Total Area Traversed: area_t (using prev nodes)
