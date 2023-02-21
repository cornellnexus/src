#import wmm2020
import csv
import math
import numpy as np
from ahrs.filters import EKF
import csv
import os

from constants.definitions import CSV_PATH


with open(CSV_PATH + '/imu_360_sample1.csv') as csv_file:
    reader = csv.reader(csv_file)
    row_count = sum(1 for row in reader)

with open(CSV_PATH + '/imu_360_sample1.csv') as csv_file:
    reader = csv.reader(csv_file)
    acc_data = np.zeros((row_count, 3))
    mag_data = np.zeros((row_count, 3))
    gyr_data = np.zeros((row_count, 3))
    row_num = 0

    for row in reader:
        acc_x = float(row[0][14:])
        acc_y = float(row[1][6:])
        acc_z = float(row[2][6:-1])

        mag_x = float(row[3][13:])
        mag_y = float(row[4][6:])
        mag_z = float(row[5][6:-1])

        gyr_x = float(row[6][14:])
        gyr_y = float(row[7][6:])
        gyr_z = float(row[8][5:-2])
        heading = np.degrees(math.atan2(mag_y, mag_x))
        # print(heading)
        acc_data[row_num] = np.array([acc_x, acc_y, acc_z])
        mag_data[row_num] = np.array([mag_x, mag_y, mag_z])
        gyr_data[row_num] = np.array([gyr_x, gyr_y, gyr_z])

        row_num += 1

# print(acc_data)
# print(mag_data)
# print(gyr_data)

ekf = EKF()
ekf = EKF(gyr=gyr_data, acc=acc_data, mag=mag_data)
# print(ekf.R)
# print(np.shape(ekf.R))
height = np.shape(ekf.R)[0]
for i in range(height):
    top = 2.0 * (ekf.R[i, 2] * ekf.R[i, 3] + ekf.R[i, 0] * ekf.R[i, 1])
    bottom = ekf.R[i, 0] * ekf.R[i, 0] - ekf.R[i, 1] * ekf.R[i, 1] - ekf.R[i, 2] * ekf.R[i, 2] + ekf.R[i, 3] * ekf.R[
        i, 3]

    heading = np.degrees(math.atan2(top, bottom))

    # ekf.R[i,1] = 0;
    # ekf.R[i,3] = 0;
    # m = math.sqrt(ekf.R[i,0]*ekf.R[i,0] + ekf.R[i,2]*ekf.R[i,2]);
    # ekf.R[i,0] /= m
    # ekf.R[i,2] /= m;
    # heading = np.degrees(2*math.acos(ekf.R[i,0]));
    # print(heading)
