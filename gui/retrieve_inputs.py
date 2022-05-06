import os
import sys
import statistics
from engine.packet import *
from gui.robot_data import get_values, get_integer_value, get_float_value


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
    count = 0
    for packet in packets:
        try:
            packet_data = packet.split(";")
            phases.append(get_integer_value(packet_data[0]))
            weights.append(get_float_value(packet_data[1]))
            accs.append(get_values(packet_data[2],3))
            n_dists.append(get_float_value(packet_data[3]))
            rots.append(get_float_value(packet_data[4]))
            last_ns.append(get_values(packet_data[5],2))
            vels.append(get_float_value(packet_data[6]))
            next_ns.append(get_values(packet_data[7],2))
            coords.append(get_values(packet_data[8],2))
            batts.append(get_integer_value(packet_data[9]))
            ctrls.append(get_integer_value(packet_data[10]))
            count = count + 1
        except:
            pass


    if(count != 0):
        phase = get_mode(phases)
        weight = get_median(weights)
        acc = get_medians(accs)
        n_dist = get_median(n_dists)
        rot = get_median(rots)
        last_n = get_medians(last_ns)
        vel = get_median(vels)
        next_n = get_medians(next_ns)
        coord = get_medians(coords)
        batt = get_median(batts)
        ctrl = get_mode(ctrls)
    
        # return packet with combined data --> need to extend or shrink value to match data string
        return str(Packet(phase, weight, acc, n_dist, rot, last_n, vel, next_n, coord, batt, ctrl))
    else:
        raise Exception("Not enough data to validate.")


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


def get_medians(data):
    '''

    Args:
        data: list of list of floats
        num_inputs: number of inputs


    Returns: a tuple of strings where the first and second entries are the medians of [coords]

    '''
    parameters = [] # [[],[],[]]
    parameter_medians = []
    num_inputs = len(data[0])
    for i in range(num_inputs):
        parameters.append([])

    for packet in data: #[8.01, 0.01, 0.0]
        for i in range(num_inputs):
            parameters[i].append(packet[i])

    for i in range(num_inputs):
        parameter_medians.append(get_median(parameters[i]))

    return parameter_medians


# update_gui()
