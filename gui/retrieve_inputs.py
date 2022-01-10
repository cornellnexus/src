import os
import sys
import serial
import statistics
from engine.packet import Packet
# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)


def update_gui():
    '''

    Reads telemetry data packets from raspberry pi, which ideally of the following format
    "phase:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coors:000.00,000.00;batt:000;ctrl:1"

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

        robot_data_file.write(str(valid_packet) + '\n')
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
    return Packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl)


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
