import serial
import statistics 
import time

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
    while True:
        while len(packets) < 5: 
            packet = ser.readline()
            if 80 < len(packet) < 150: #check if packet length is appropriate
                packets.append(packet)
        valid_packet = validate_packet(packets)


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
        phases.append(packet[0])
        weights.append(packet[1])
        accs.append(packet[2])
        n_dists.append(packet[3])
        rots.append(packet[4])
        last_ns.append(packet[5])
        vels.append(packet[6])
        next_ns.append(packet[7])
        coords.append(packet[8])
        batts.append(packet[9])

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

    pass 
    # return packet with combined data --> need to extend or shrink value to match data string 

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
    #[SETUP: 0, AVOID_OBSTACLE: 1, RETURN: 2, DOCKING: 3, COMPLETE: 4]
# Pounds of Collected Plastic: p_weight
# Acceleration: acc (imu)
# Current Distance to Next Node: n_dist (mission)
# Rotation: rot (imu)
# Last Node Visited: last_n (mission)
# Velocity: vel (imu)
# Next Node to Visit: next_n (mission)
# Current Coordinates: coords
# Battery level: bat


    #send 5 packets, then pause, then repeat/continue
    # "phse:0;p_weight:00.0;acc:0.00;n_dist:00.0;rot:00.00;last_n:000.00,000.00;vel:0.00;next_n:000.00,000.00;coords:000.00,000.00;bat:000"
    ### preset length: 131 characters


    # case by case check each parameter with each other in helpers;
    # phase: for each check valid number, remove invalids, take phase with max freq

    #p_weight, acc, dist, rot, vel, bat: median

    #last_n, next_n, coords: median of x and y


    # return final_packet
#


### Port
# 017543DC

### Structure
# "phse:stuff,acc: stuf"
# "id:data1,next_id:data2,...,last_id:data3"

#### From raspberry pi
# Robot Phase: phse (mission)
    #[SETUP: 0, AVOID_OBSTACLE: 1, RETURN: 2, DOCKING: 3, COMPLETE: 4]
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