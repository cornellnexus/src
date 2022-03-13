import serial

ser = serial.Serial('/dev/tty.usbserial-017543DC', 57600)

while True: 
    data = '000 \n'
    cast_data = bytes(data, encoding = 'utf-8')
    ser.write(cast_data)
    # data = ser.read()
    print(cast_data)