import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
import math
import ast
from engine.sensor_module import SensorModule

if __name__ == "__main__":

    # Determine data read mode
    IS_LIVE_DATA = False
    live_data = int(input("1: Visualize heading using live data from an IMU\n2: Visualize heading using pre-collected "
                          "data stored in a CSV file\nEnter your choice: "))
    if live_data != 1 and live_data != 2:
        raise Exception("Invalid input.")
    IS_LIVE_DATA = live_data == 1

    # Plot setup
    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
    circle_patch = plt.Circle((0, 0), 1, fc="green")
    wedge_patch = patch.Wedge(
        (0, 0), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx")

    # Data reading setup
    if IS_LIVE_DATA:
        sensor_module = SensorModule(write=True)


    else:
        with open("csv/imu_360_sample1.csv") as file:
            imu_readings = file.readlines()
            imu_readings = [ast.literal_eval(line) for line in imu_readings]


    def init():
        circle_patch.center = (0, 0)
        ax.add_patch(circle_patch)
        ax.add_patch(wedge_patch)
        return circle_patch, wedge_patch


    def animate(i):
        if IS_LIVE_DATA:
            sensor_module.update_imu_data()
            middle_heading = math.degrees(math.atan2(sensor_module.imu_dict["mag_y"], sensor_module.imu_dict["mag_x"]))
        else:
            middle_heading = math.degrees(math.atan2(imu_readings[i]["mag"]["y"], imu_readings[i]["mag"]["x"]))

        wedge_patch.update({"center": [0, 0]})
        wedge_patch.theta1 = middle_heading - 10
        wedge_patch.theta2 = middle_heading + 10
        # time.sleep(0.1)
        return circle_patch, wedge_patch


    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(imu_readings) - 1, interval=20, blit=True)
    plt.show()
