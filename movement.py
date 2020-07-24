# floating point inaccuracies?
import math
import csv
import time
from pynput import keyboard


class Commands:
    # does this work for printing first row with the column names?
    def print_coords(self):
        with open("coordinates.txt") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                print("x: " + row[0] + "\ty: " + row[1] +
                      "\theading: " + row[2])

    def write_coords(self):
        with open("coordinates.txt", "a", newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(
                ["%.2f" % self.x, "%.2f" % self.y, "%.2f" % self.heading])

    # coords as floats, angles in degrees
    def __init__(self, start_x, start_y, start_d):
        self.x = float(start_x)
        self.y = float(start_y)
        self.heading = start_d % 360.0
        self.write_coords()
        # arbitrary values, constant variables
        self.radius = 4
        self.time_step = 0.01

        # wheel rpm's
        self.front_left_rpm = 0
        self.front_right_rpm = 0
        self.back_left_rpm = 0
        self.back_right_rpm = 0

    def calc_steps(self, dist, rpm):
        circumference = self.radius*2*math.pi
        min_num = (dist/circumference)/rpm
        sec_num = min_num * 60
        return sec_num/self.time_step

    def move_in_line(self, dist, rpm):

        self.front_left_rpm = rpm
        self.front_right_rpm = rpm
        dist_per_time_step = (rpm/60)*self.time_step*self.radius*2*math.pi
        steps_to_travel = self.calc_steps(dist, rpm)

        while (steps_to_travel > 0):
            self.x += float(dist_per_time_step) * \
                math.sin(math.radians(self.heading))
            self.y += float(dist_per_time_step) * \
                math.cos(math.radians(self.heading))
            self.write_coords()
            steps_to_travel -= 1
            time.sleep(self.time_step)
        self.front_left_rpm = 0
        self.front_right_rpm = 0

    def move_with_key(self, dist):
        self.x += float(dist) * \
            math.sin(math.radians(self.heading))
        self.y += float(dist) * \
            math.cos(math.radians(self.heading))
        self.write_coords()
        self.print_coords()
        # self.write_coords()

    def turn_with_key(self, deg):
        self.heading = (self.heading + deg) % 360.0
        self.write_coords()
        self.print_coords()
        # self.write_coords()

    #deg in degrees

    def turn_in_place(self, deg):
        # change rpm for each of the wheels, for a certain amount of time
        # until the robot has turned deg degrees?

        self.heading = (self.heading + deg) % 360.0
        self.write_coords()

    def turn_around(self):
        self.turn_in_place(180.0)
        self.write_coords()


# c = Commands(0.0, 0.0, 0.0)
# c.turn_in_place(45)
# c.move_in_line(50.0, 30)
# c.print_coords()

# With time step .1
# 0.0,47.752208334564855,0.0
# 0.0,49.00884539600077,0.0
# 0.0,50.26548245743669,0.0

# With time step .05
# 0.0, 49.008845396000865, 0.0
# 0.0, 49.63716392671883, 0.0
# 0.0, 50.26548245743679, 0.0
