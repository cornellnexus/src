import serial
import time

data = {"phse": "Traversal",
        "p_weight": "10",
        "acc":"25",
        "n_dist": "100",
        "area_t": "30",
        "rot": "20",
        "last_n": "("
        }
ser = serial.Serial("/dev/cu.usbserial-017543DC", 57600)


num_data = 0
grouping = [0,1,2] #packets
while True:

    # Readlines

    packet = ser.readline()
    print(packet)
    packet_len = len(packet)
    if num_data
    if packet_len > 80 and packet_len < 150:
        # Potentially viable data
        data_points = packet_len.split(";")
        # next_packet = valid_packet(grouping) #single packet to add to csv
        #add next_packet to csv :)


        # separator = packet.index(":")
        # id = packet[0: separator]
        # data = packet[separator+2:]

        # error

    num_data = num_data

def valid_packet(grouping):
    packet1 = grouping[0]
    packet2 = grouping[1]
    packet3 = grouping[2] #change to 5
    packet4 = grouping[3]
    packet5 = grouping[4]
    pass
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