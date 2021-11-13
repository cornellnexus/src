import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import animation as animation
import ast

from engine.kinematics import get_vincenty_x, get_vincenty_y
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
    eng_quad = [(42.444227, -76.484059), (42.445147, -76.482492,)]  # Lat, long
    x_range = get_vincenty_x(eng_quad[0], eng_quad[1])
    y_range = get_vincenty_y(eng_quad[0], eng_quad[1])
    print("x_range", x_range, "y_range", y_range)

    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)

    ax = plt.axes(xlim=(0, x_range), ylim=(0, y_range))
    circle_patch = plt.Circle((0, 0), 1, fc="red")
    eng_quad_img = mpimg.imread("geo_images/engineering_quad.png")
    plt.imshow(eng_quad_img, origin="upper", extent=[0, x_range, 0, y_range])

    if IS_LIVE_DATA:
        sensor_module = SensorModule(write=True)
        frames = 10
    else:
        with open("data/GPS_13-11-2021_14:18:15.txt") as file:
            gps_readings = file.readlines()
            gps_readings = [ast.literal_eval(line) for line in gps_readings]
        frames = len(gps_readings) - 1

    def init():
        circle_patch.center = (0, 0)
        ax.add_patch(circle_patch)
        return [circle_patch]


    def animate(i):
        if IS_LIVE_DATA:
            sensor_module.update_gps_data()
            new_location = sensor_module.get_measurement(eng_quad[0])[:2]

        else:
            gps_coord = (gps_readings[i]["lat"], gps_readings[i]["lon"])
            new_location = (get_vincenty_x(eng_quad[0], gps_coord), get_vincenty_y(eng_quad[0], gps_coord))

        circle_patch.center = new_location
        return [circle_patch]


    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=20, blit=True)
    plt.show()
