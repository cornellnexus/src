import serial
import time

'''
Handshake script to be run by the robot. Should be run slightly before running the laptop handshake script.
Establishes a handshake by decoding a message sent by the computer and then sends a message to the laptop
to make a two way connection.
'''

# Initializes port and clears any exisiting messages
port = serial.Serial(port="/dev/serial0", baudrate=57600, timeout=0)
port.flush()
port.flushInput()
print("RPI script begins")
# Waits 5 seconds to see if port recieves any message
t_end = time.time() + 5
line = None
while time.time() < t_end:
    if port.in_waiting > 0:
        # TODO: Check decoding
        line = port.readline().decode("utf-8")
        print(line) # Delete after done testing
        break

if not line:
    print("No message recieved")

if (line == "computer_to_robot"):
    # Send message to laptop to confirm handshake
    stringy = "robot_to_computer"
    # TODO: Check if casting to bytes / encoding is necessary
    cast_stringy = bytes(stringy, encoding = "utf-8")
    port.write(cast_stringy)

    port.flush() # Unsure if needed
    port.flushInput() # Unsure if needed
    t_end = time.time() + 5
    line = None

    while time.time() < t_end:
        if port.in_waiting > 0: 
            line = port.readline().decode("utf-8")
            print(line) # Delete after done testing
            break

    if(line == "success"):
        print("Success")
    else:
        print("Failure")
else:
    print("Failure - Line is \"" + str(line) + "\" but should be computer_to_robot")