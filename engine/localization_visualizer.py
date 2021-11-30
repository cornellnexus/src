import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import animation as animation
from matplotlib import patches as patch
import ast
import sys
import math
from enum import IntEnum

from engine.kinematics import get_vincenty_x, get_vincenty_y
from engine.sensor_module import SensorModule
from constants.geo_fences import ENGINEERING_QUAD


class DataType(IntEnum):
    IMU_ONLY = 1
    GPS_ONLY = 2
    IMU_AND_GPS = 3


"""
Command-line arguments:
live - 0 if using data from file, 1 if using data from active sensor
data - 0 if heading visualization desired (IMU), 1 if position visualization desired (GPS), 
       2 if both heading and location visualization ddesired (IMU and GPS)
ekf -  0 if raw measurements should be visualized, 1 if EKF should be used
"""
if __name__ == "__main__":
    # Settings, can be changed as desired
    zone = ENGINEERING_QUAD  # Used for GPS visualization
    zone_photo = "geo_images/engineering_quad.png"
    imu_data_file = "data/imu_360_sample1.csv"
    gps_data_file = "data/GPS_22-11-2021.txt"

    # Read command-line arguments
    if len(sys.argv) != 4 or "live=" not in sys.argv[1] or "data=" not in sys.argv[2] or "ekf=" not in sys.argv[3]\
            or len(sys.argv[1]) != 6 or len(sys.argv[2]) != 6 or len(sys.argv[3]) != 5:
        sys.exit("Two inputs required, and must be of this format: live=<0 (Live data) or 1 (Data from file)> and "
                 "data=<0 (IMU) or 1 (GPS) or 2 (IMU and GPS)>")

    is_live = sys.argv[1][sys.argv[1].index("=")+1:]
    if is_live != "0" and is_live != "1":
        sys.exit("The value of the live argument must be 0 or 1.")
    is_live = int(is_live)

    data_type = sys.argv[2][sys.argv[2].index("=")+1:]
    if data_type != "1" and data_type != "2" and data_type != "3":
        sys.exit("The value of the data argument must be 0, 1, or 2.")
    data_type = int(data_type)

    use_ekf = sys.argv[3][sys.argv[3].index("=")+1:]
    if use_ekf != "0" and use_ekf != "1":
        sys.exit("The value of the ekf argument must be 0 or 1.")
    use_ekf = int(use_ekf)

    # Plot setup
    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)

    if data_type == DataType.IMU_ONLY:
        ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
    else:
        x_range = get_vincenty_x(zone[0], zone[1])
        y_range = get_vincenty_y(zone[0], zone[1])
        ax = plt.axes(xlim=(0, x_range), ylim=(0, y_range))

    circle_patch = plt.Circle((0, 0), 1, fc="blue")
    if data_type == DataType.IMU_ONLY or data_type == DataType.IMU_AND_GPS:
        wedge_patch = patch.Wedge(
            (0, 0), 3, 100, 80, animated=True, fill=False, width=2, ec="b", hatch="xx")
    if data_type == DataType.GPS_ONLY or data_type == DataType.IMU_AND_GPS:
        eng_quad_img = mpimg.imread(zone_photo)
        plt.imshow(eng_quad_img, origin="upper", extent=[0, x_range, 0, y_range])

    # Sensor data acquisition setup
    if is_live:
        sensor_module = SensorModule(write=False)
        frames = 10
    else:
        imu_readings = None
        gps_readings = None

        if data_type == DataType.IMU_ONLY:
            with open(imu_data_file) as file:
                imu_readings = file.readlines()
                imu_readings = [ast.literal_eval(line) for line in imu_readings]
        elif data_type == DataType.GPS_ONLY:
            with open(gps_data_file) as file:
                gps_readings = file.readlines()
                gps_readings = [ast.literal_eval(line) for line in gps_readings]
        else:
            with open(imu_data_file) as file:
                imu_readings = file.readlines()
                imu_readings = [ast.literal_eval(line) for line in imu_readings]
            with open(gps_data_file) as file:
                gps_readings = file.readlines()
                gps_readings = [ast.literal_eval(line) for line in gps_readings]

        if not gps_readings:
            frames = len(imu_readings)
        elif not imu_readings:
            frames = len(gps_readings)
        else:
            frames = min(len(imu_readings) - 1, len(gps_readings) - 1)

    # EKF setup


    def init():
        circle_patch.center = (0, 0)
        if data_type == DataType.GPS_ONLY:
            ax.add_patch(circle_patch)
            return [circle_patch]
        else:
            ax.add_patch(circle_patch)
            ax.add_patch(wedge_patch)
            return circle_patch, wedge_patch


    def animate(i):
        if data_type == DataType.IMU_ONLY:
            if is_live:
                sensor_module.update_imu_data()
                new_heading = math.degrees(
                    math.atan2(sensor_module.imu_dict["mag"][1], sensor_module.imu_dict["mag"][0]))
            else:
                new_heading = math.degrees(math.atan2(imu_readings[i]["mag"]["y"], imu_readings[i]["mag"]["x"]))
            wedge_patch.update({"center": (0, 0)})
            wedge_patch.theta1 = new_heading - 10
            wedge_patch.theta2 = new_heading + 10
            return circle_patch, wedge_patch

        elif data_type == DataType.GPS_ONLY:
            if is_live:
                sensor_module.update_gps_data()
                new_location = sensor_module.get_measurement(zone[0])[:2]
            else:
                gps_coord = (gps_readings[i]["lat"], gps_readings[i]["lon"])
                new_location = (get_vincenty_x(zone[0], gps_coord), get_vincenty_y(zone[0], gps_coord))

            circle_patch.center = new_location
            return [circle_patch]

        elif data_type == DataType.IMU_AND_GPS:
            if is_live:
                sensor_module.update_imu_data()
                sensor_module.update_gps_data()
                measurements = sensor_module.get_measurement(zone[0])
                new_heading = measurements[2]
                new_location = (measurements[0], measurements[1])

            else:
                new_heading = math.degrees(math.atan2(imu_readings[i]["mag"]["y"], imu_readings[i]["mag"]["x"]))
                gps_coord = (gps_readings[i]["lat"], gps_readings[i]["lon"])
                new_location = (get_vincenty_x(zone[0], gps_coord), get_vincenty_y(zone[0], gps_coord))

            circle_patch.center = new_location
            wedge_patch.update({"center": new_location})
            wedge_patch.theta1 = new_heading - 10
            wedge_patch.theta2 = new_heading + 10
            return circle_patch, wedge_patch

    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=20, blit=True)
    plt.show()
