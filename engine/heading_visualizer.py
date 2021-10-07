import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
import math
import ast
import time
from engine.sensor_module import SensorModule

if __name__ == "__main__":

    # Plot setup
    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)

    ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))

    circle_patch = plt.Circle((0, 0), 1, fc="green")
    wedge_patch = patch.Wedge(
        (0, 0), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    )

    # Sensor setup
    # sensor_module = SensorModule()
    with open("csv/imu_360_sample1.csv") as file:
        lines = file.readlines()
        # lines = [line.rstrip() for line in lines]
        lines = [ast.literal_eval(line) for line in lines]

    print(lines)

    def init():
        circle_patch.center = (0, 0)
        ax.add_patch(circle_patch)
        ax.add_patch(wedge_patch)
        return circle_patch, wedge_patch


    def animate(i):
        # sensor_module.update_imu_data()

        middle_heading = math.degrees(math.atan2(lines[i]["mag"]["y"], lines[i]["mag"]["x"]))
        wedge_patch.update({"center": [0, 0]})
        wedge_patch.theta1 = middle_heading - 10
        wedge_patch.theta2 = middle_heading + 10
        # time.sleep(0.1)
        print(i)
        return circle_patch, wedge_patch


    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(lines)-1, interval=20, blit=True)
    plt.show()
