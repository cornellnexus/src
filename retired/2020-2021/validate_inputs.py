import os
import sys

import serial
import statistics
import time

from constants.definitions import CSV_PATH

# data = {"phse": "Traversal",
#         "p_weight": "10",
#         "acc":"25",
#         "n_dist": "100",
#         "area_t": "30",
#         "rot": "20",
#         "last_n": "("
#         }
ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)


def update_gui():
    packets = []
    robot_data_file = open((CSV_PATH + '/robot_data.csv'), "r+")  # open csv file of robot data
    while True:
        while len(packets) < 5:
            packet = ser.readline()
            if 80 < len(packet) < 150:  # check if packet length is appropriate
                packets.append(packet)
        valid_packet = validate_packet(packets)

        robot_data_file.write(valid_packet + '\n')

def get_values(data):
    '''
    Args:
        data: a string of format "id:value"

    Returns: value (substring after :)
    '''
    separator_index = data.find(":")
    if separator_index == -1:
        print("data corruption")
        raise Exception()
    return data[separator_index + 1:]


def validate_packet(packets):
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

    for packet in packets:
        packet_data = packet.split(";")
        phases.append(get_values(packet_data[0]))
        weights.append(get_values(packet_data[1]))
        accs.append(get_values(packet_data[2]))
        n_dists.append(get_values(packet_data[3]))
        rots.append(get_values(packet_data[4]))
        last_ns.append(get_values(packet_data[5]))
        vels.append(get_values(packet_data[6]))
        next_ns.append(get_values(packet_data[7]))
        coords.append(get_values(packet_data[8]))
        batts.append(get_values(packet_data[9]))

    phase = get_phase(phases)
    weight = get_median(weights)
    acc = get_median(accs)
    n_dist = get_median(n_dists)
    rot = get_median(rots)
    last_n = get_coord(last_ns)
    vel = get_median(vels)
    next_n = get_coord(next_ns)
    coord = get_coord(coords)
    batt = get_median(batts)

    # return packet with combined data --> need to extend or shrink value to match data string
    return build_validated_packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt)


def build_validated_packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt):
    # "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000"
    return "phse" + phase + ";p_weight" + fix_data_size(weight, 2, 1) + ";acc" + fix_data_size(acc, 1, 2) + ";n_dist" + \
           fix_data_size(n_dist, 2, 1) + ";rot" + fix_data_size(rot, 2, 2) + ";last_n" + fix_tuple_size(last_n, 3, 2) + \
           ";vel" + fix_data_size(vel, 1, 2) + ";next_n" + fix_tuple_size(next_n, 3, 2) + \
           ";coords" + fix_tuple_size(coord, 3, 2) + ";bat" + + fix_data_size(batt, 3, 0)


def fix_data_size(s, desired_int_length, desired_decimal_length):
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
        s = s[0:len(s) - deci_difference]
    else:
        # extend: add 0's to the end to extend decimal values
        deci_buffer = "0" * deci_difference
        s = s + deci_buffer

    return s


def fix_tuple_size(t, desired_int_length, desired_decimal_length, ):
    return (fix_data_size(t[0], desired_int_length, desired_decimal_length), \
            fix_data_size(t[1], desired_int_length, desired_decimal_length))


def get_phase(phases):
    processed_phases = [x for x in phases if x <= 4]
    final_phase = statistics.mode(processed_phases)
    return str(final_phase)


def get_median(data_list):
    return str(statistics.median(data_list))


def get_coord(coords):
    x = []
    y = []
    for coord in coords:
        x.append(coord[0])
        y.append(coord[1])
    x_median = get_median(x)
    y_median = get_median(y)
    return (str(x_median), str(y_median))

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


# send 5 packets, then pause, then repeat/continue
# "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000"
### preset length: 131 characters


# case by case check each parameter with each other in helpers;
# phase: for each check valid number, remove invalids, take phase with max freq

# p_weight, acc, dist, rot, vel, bat: median

# last_n, next_n, coords: median of x and y


# return final_packet
#


### Port
# 017543DC

### Structure
# "phse:stuff,acc: stuf"
# "id:data1,next_id:data2,...,last_id:data3"

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


# the plan:
## Read 1 line/packet each with 10 ids, aim for 3x each
## Set up csv for current info -- OMIT
## Error checking
## Send correct terms to another csv
## read cleaned up csv in gui
