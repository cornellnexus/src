from engine.robot import Phase

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

class RobotData(object):
    """
    A class that represents the current robot data

    Attributes:
        phase: Current phase value [int from 0 - 6]
        weight: Weight of collected plastic [float > 0]
        acc: Acceleration [float > 0]
        n_dist: Distance to next node [float > 0]
        rot: Rotation [float > 0]
        last_n: Coordinates of last node visited [tuple of float > 0]
        vel: Velocity [float > 0]
        next_n: Coordinates of the next node to visit [tuple of float > 0]
        coord: Coordinates of the robot [tuple of float > 0]
        bat: Remaining battery percentage [int > 0]
        ctrl: Robot control mode [int from 1-5]
    """

    def update_data(self, packet):
        packet_data = packet.split(";")
        self.phase = get_integer_value(packet_data[0])
        self.weight = get_float_value(packet_data[1])
        self.acc = get_float_value(packet_data[2])
        self.n_dist = get_float_value(packet_data[3])
        self.rot = get_float_value(packet_data[4])
        self.last_n = get_tuple_value(packet_data[5])
        self.vel = get_float_value(packet_data[6])
        self.next_n = get_tuple_value(packet_data[7])
        self.coord = get_tuple_value(packet_data[8])
        self.bat = get_integer_value(packet_data[9])
        self.ctrl = get_integer_value(packet_data[10])

        # calculate total area traversed

    def __init__(self, packet):
        assert type(packet) == str

        packet_data = packet.split(";")
        self.phase = get_integer_value(packet_data[0])
        self.weight = get_float_value(packet_data[1])
        self.acc = get_float_value(packet_data[2])
        self.n_dist = get_float_value(packet_data[3])
        self.rot = get_float_value(packet_data[4])
        self.last_n = get_tuple_value(packet_data[5])
        self.vel = get_float_value(packet_data[6])
        self.next_n = get_tuple_value(packet_data[7])
        self.coord = get_tuple_value(packet_data[8])
        self.bat = get_integer_value(packet_data[9])
        self.ctrl = get_integer_value(packet_data[10])

    def __str__(self):
        p = str(Phase(self.phase))
        return "Robot Phase: " + p[p.find(".")+1:] + \
               "\nPounds of Collected Plastic: " + str(self.weight) + "g"+ \
               "\nAcceleration: " + str(self.acc) + f" m/s\N{SUPERSCRIPT TWO}" + \
               "\nCurrent Distance to Next Node: " + str(self.n_dist) + \
               "\nRotation: " + str(self.rot) + \
               "\nLast Node Visited: " + str(self.last_n) + \
               "\nVelocity: " + str(self.vel) + " m/s"\
               "\nNext Node to Visit: " + str(self.next_n) + \
               "\nCurrent Coordinates: " + str(self.coord) + \
               "\nBattery Level: " + str(self.bat) + \
               "\nControl Mode: " + str(self.ctrl) 

        # "\nTotal Area Traversed: " + self.area
