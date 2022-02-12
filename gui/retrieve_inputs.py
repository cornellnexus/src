import os
import sys
import serial
import statistics
from engine.packet import *
from gui.robot_data import get_tuple_value, get_integer_value, get_float_value
# ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)


def update_gui():
    '''

    Reads telemetry data packets from raspberry pi, which ideally of the following format
    "phase:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coors:000.00,000.00;batt:000;ctrl:1"

    From every 5 data packets, a single data packet is created to estimate the packets. This single data packet
    is written to robot_data.csv, so we can accurately display current information regarding the robot on the GUI.

    '''
    packets = []

    rpi_to_gui = open((get_path('csv')[-1] + '/rpi_to_gui_simulation.csv'), "r")  # open csv file of rpi to gui data
    print("starting retrieve inputs")
    # robot_data_file.write("start\n")
    while True:
        while len(packets) < 5:
            # packet = ser.readline()
            # packet = input("Enter data: ") # use this for testing purposes
            try:
                packet = rpi_to_gui.readlines()[-1]  # get last line of csv file
                print("packet is " + packet)
                if 80 < len(packet) < 150:  # check if packet length is appropriate
                    packets.append(packet)
                    print("appending packet")
                    # robot_data_file.write("test\n")
            except:
                pass
        print("validating packet")
        valid_packet = validate_packet(packets)
        robot_data_file = open((get_path('csv')[-1] + '/robot_data.csv'), "a")  # open csv file of robot data
        robot_data_file.write(valid_packet + '\n')
        robot_data_file.close()
        print("write " + valid_packet + " to csv")
        packets = []

    rpi_to_gui.close()



def get_path(folder):
    '''
    Args:
        folder: a string of folder directory

    Returns: sys path to desired folder
    '''
    cwd = os.getcwd()
    sys.path.append(cwd + "/" + folder)
    return sys.path



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
    return str(Packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl))


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


# update_gui()
