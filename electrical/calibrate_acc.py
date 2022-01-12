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

    global running_amin
    global running_amax
    # print(running_amin)
    # print(running_amax)

    # sensor = LSM9DS1_I2C(i2c)
    accel_x, accel_y, accel_z = sensor.acceleration

    accel_tuple = (accel_x, accel_y, accel_z)

    time.sleep(3)


    running_amin = tuple(map(lambda x, y: min(x,y), running_amin, accel_tuple))
    running_amax = tuple(map(lambda x, y: max(x,y), running_amax, accel_tuple))

    # print(running_amin)
    # print(running_amax)
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
    #
    # print(aoffset)
    # print(scale_ax)
    # print(scale_ay)
    # print(scale_az)
    # print("----------")

    return (aoffset, scale_ax, scale_ay, scale_az)

def accel_cal():
    accel_cal_val = accel_cal_aux()
    aoffsetx = accel_cal_val[0][0]
    aoffsety = accel_cal_val[0][1]
    aoffsetz = accel_cal_val[0][2]
    # print("aoffset: " + str(accel_cal_val[0]))

    scale_ax = accel_cal_val[1]
    scale_ay = accel_cal_val[2]
    scale_az = accel_cal_val[3]

    sensor_ax, sensor_ay, sensor_az = sensor.acceleration

    corrected_ax = (sensor_ax - aoffsetx) * scale_ax
    corrected_ay = (sensor_ay - aoffsety) * scale_ay
    # print("sensor_acc: " + str((sensor_ax, sensor_ay, sensor_az)))
    corrected_az = (sensor_az - aoffsetz) * scale_az

    corrected_accel = (corrected_ax, corrected_ay, corrected_az, corrected_az + 9.81)

    # print("corrected_accel: " + str(corrected_accel))
    return corrected_accel

def main():
    sensor_ax, sensor_ay, sensor_az = sensor.acceleration
    print("sensor_acc: " + str((sensor_ax, sensor_ay, sensor_az)))

    while (len(corrected_accel_lst) < 10):
        updated_accel = accel_cal()
        corrected_accel_lst.append(updated_accel)
    print(corrected_accel_lst)

# if __name__ == '__main__':
#     main()



# def mag_cal():
#
#     magnetometer = LSM9DS1_I2C(i2c)
#
#     time.sleep(3)
#
#     mag_x, mag_y, mag_z = magnetometer.magnetic
#     min_x = max_x = mag_x
#     min_y = max_y = mag_y
#     min_z = max_z = mag_z
#
#     t_end = time.time() + 60
#     while time.time() < t_end:
#         mag_x, mag_y, mag_z = magnetometer.magnetic
#
#         min_x = min(min_x, mag_x)
#         min_y = min(min_y, mag_y)
#         min_z = min(min_z, mag_z)
#
#         max_x = max(max_x, mag_x)
#         max_y = max(max_y, mag_y)
#         max_z = max(max_z, mag_z)
#
#         # Get the hard-iron offset values
#         HI_offset_x = (max_x + min_x) / 2
#         HI_offset_y = (max_y + min_y) / 2
#         HI_offset_z = (max_z + min_z) / 2
#
#         # Get the soft-iron offset values
#         SI_offset_x = (max_x - min_x) / 2
#         SI_offset_y = (max_y - min_y) / 2
#         SI_offset_z = (max_z - min_z) / 2
#
#     return (HI_offset_x, HI_offset_y, HI_offset_z, SI_offset_x, SI_offset_y, SI_offset_z)
#
#
#
# def gyr_cal():
#
#     gyro_accel = LSM9DS1_I2C(i2c)
#
#     time.sleep(3)
#
#     gyro_x, gyro_y, gyro_z = gyro_accel.gyro
#     min_x = max_x = gyro_x
#     min_y = max_y = gyro_y
#     min_z = max_z = gyro_z
#
#     t_end = time.time() + 60
#     while time.time() < t_end:
#         gyro_x, gyro_y, gyro_z = gyro_accel.gyro
#
#         min_x = min(min_x, gyro_x)
#         min_y = min(min_y, gyro_y)
#         min_z = min(min_z, gyro_z)
#
#         max_x = max(max_x, gyro_x)
#         max_y = max(max_y, gyro_y)
#         max_z = max(max_z, gyro_z)
#
#         offset_x = (max_x + min_x) / 2
#         offset_y = (max_y + min_y) / 2
#         offset_z = (max_z + min_z) / 2
#
#     return (offset_x, offset_y, offset_z)