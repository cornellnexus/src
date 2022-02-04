import time
import board
import busio
from adafruit_lsm9ds1 import LSM9DS1_I2C


i2c = busio.I2C(board.SCL, board.SDA)
sensor = LSM9DS1_I2C(i2c)
running_amin = (32767, 32767, 32767)
running_amax = (-32768, -32768, -32768)

corrected_accel_lst = []


def accel_cal_aux():
    """
    Returns tuple containing (aoffset, scale_ax, scale_ay, scale_az)
    Helper function for [accel_cal] in getting the IMU accelerometer's raw data values in 
    x, y, and z directions, as well as determining the offset error for the accelerometer.

    Parameters: None.

    Precondition: None.
    """
    global running_amin
    global running_amax

    accel_x, accel_y, accel_z = sensor.acceleration

    accel_tuple = (accel_x, accel_y, accel_z)

    time.sleep(3)


    running_amin = tuple(map(lambda x, y: min(x,y), running_amin, accel_tuple))
    running_amax = tuple(map(lambda x, y: max(x,y), running_amax, accel_tuple))

    t_end = time.time() + 60
    while time.time() < t_end:
        accel_x, accel_y, accel_z = sensor.acceleration
        accel_tuple = (accel_x, accel_y, accel_z)
        running_amin = tuple(map(lambda x, y: min(x,y), running_amin, accel_tuple))
        running_amax = tuple(map(lambda x, y: max(x,y), running_amax, accel_tuple))

    aoffset = tuple(map(lambda x1, x2: (x1+x2) / 2., running_amin, running_amax))
    avg_adelta = tuple(map(lambda x1, x2: (x2-x1)/2., running_amin, running_amax))
    combined_avg_adelta = (avg_adelta[0] + avg_adelta[1] + avg_adelta[2])/3.
    scale_ax = combined_avg_adelta / avg_adelta[0]
    scale_ay = combined_avg_adelta / avg_adelta[1]
    scale_az = combined_avg_adelta / avg_adelta[2]

    return (aoffset, scale_ax, scale_ay, scale_az)

def accel_cal():
    """
    Returns tuple containing (corrected_ax, corrected_ay, corrected_az)
    This functions purpose is to adjust the acceleration in the x, y, and z directions 
    using the IMU error offset calculated using [accel_cal_aux()]

    Parameters: None.

    Precondition: None.
    """
    accel_cal_val = accel_cal_aux()
    aoffsetx = accel_cal_val[0][0]
    aoffsety = accel_cal_val[0][1]
    aoffsetz = accel_cal_val[0][2]

    scale_ax = accel_cal_val[1]
    scale_ay = accel_cal_val[2]
    scale_az = accel_cal_val[3]

    sensor_ax, sensor_ay, sensor_az = sensor.acceleration

    corrected_ax = (sensor_ax - aoffsetx) * scale_ax
    corrected_ay = (sensor_ay - aoffsety) * scale_ay
    corrected_az = (sensor_az - aoffsetz) * scale_az

    corrected_accel = (corrected_ax, corrected_ay, corrected_az)

    return corrected_accel

def main():
    sensor_ax, sensor_ay, sensor_az = sensor.acceleration
    print("sensor_acc: " + str((sensor_ax, sensor_ay, sensor_az)))

    while (len(corrected_accel_lst) < 10):
        updated_accel = accel_cal()
        corrected_accel_lst.append(updated_accel)
    print(corrected_accel_lst)

