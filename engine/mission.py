from collections import deque
from electrical.motor_controller import BasicMotorController, MotorController
from engine.robot import Phase
from electrical.radio_module import RadioModule

from engine.kinematics import get_vincenty_x, get_vincenty_y
from enum import Enum
from engine.grid import Grid


class ControlMode(Enum):
    """
    An enumeration of different control modes
    """
    LAWNMOWER = 1
    LAWNMOWER_B = 2
    SPIRAL = 3
    ROOMBA = 4
    MANUAL = 5
    STRAIGHT = 6


'''
Electrical library imports
(need these libraries to run mission out of the loop)
Commented out for simulation testing purposes
'''
# from electrical.gps import GPS
# from electrical.imu import IMU
# import serial
# import board
# import busio
# import adafruit_lsm9ds1

class Mission:
    def __init__(self, robot, base_station, init_control_mode, grid=Grid(42.444250, 42.444599, -76.483682, -76.483276),
                 allowed_dist_error=0.5, allowed_heading_error=0.1, allowed_docking_pos_error=0.1,
                 time_limit=50000, roomba_radius=20):
        """
        Arguments:
            robot: the Robot object linked to this Mission
            base_station: the BaseStation object linked to this Mission.
            init_control_mode: the traversal mode linked to this Mission.
            grid: the Grid which the robot should traverse
            allowed_dist_error: the maximum distance in meters that the robot can be from a node for the robot to
                have "visited" that node
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning
                in place.
            allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
                before it can start docking.
            time_limit: the maximum time the robot can execute roomba traversal mode
            roomba_radius: the maximum radius from the base station that the robot in roomba traversal mode can move
        
        Important: All the ports of the electrical classes (ie. Serial) need to be updated to the respective 
                    ports they are connected to on the computer running the code.
        """
        self.robot = robot
        self.grid = grid
        self.control_mode = ControlMode(init_control_mode)
        self.all_waypoints = self.grid.get_waypoints(self.control_mode)
        self.active_waypoints = self.grid.get_active_waypoints_list()
        self.inactive_waypoints = self.grid.get_inactive_waypoints_list()
        self.waypoints_to_visit = deque(self.all_waypoints)
        self.allowed_dist_error = allowed_dist_error
        if not robot.is_sim:
            self.gps_serial = serial.Serial('/dev/ttyACM0', 19200, timeout=5)
            self.robot_radio_serial = serial.Serial('/dev/ttyS0', 57600) #robot radio
            self.imu_i2c = busio.I2C(board.SCL, board.SDA)
            self.motor_controller = MotorController(robot, wheel_r = 0, vm_load1 = 1, vm_load2 = 1, L = 0, R = 0)
            self.robot_radio_session = RadioSession(self.robot_radio_serial)
            self.gps = GPS(self.gps_serial)
            self.imu = IMU(self.imu_i2c)
        self.allowed_heading_error = allowed_heading_error
        self.base_station_angle = base_station.heading
        self.allowed_docking_pos_error = allowed_docking_pos_error
        x = get_vincenty_x((grid.lat_min, grid.long_min), base_station.position)
        y = get_vincenty_y((grid.lat_min, grid.long_min), base_station.position)
        self.base_station_loc = (x, y)
        self.time_limit = time_limit
        self.roomba_radius = roomba_radius


    def execute_mission(self, database):
        """
        Activates the main control loop. Depending on the robot's phase, different motion control algorithms are
        activated.
        """
        while self.robot.phase != Phase.COMPLETE:
            if self.robot.phase == Phase.SETUP:
                self.robot.execute_setup(self.robot_radio_session, self.gps, self.imu, self.motor_controller)

            elif self.robot.phase == Phase.TRAVERSE:
                self.waypoints_to_visit = self.robot.execute_traversal(self.waypoints_to_visit,
                                                                       self.allowed_dist_error, self.base_station_loc,
                                                                       self.control_mode, self.time_limit,
                                                                       self.roomba_radius, database)

            elif self.robot.phase == Phase.AVOID_OBSTACLE:
                self.robot.execute_avoid_obstacle()

            elif self.robot.phase == Phase.RETURN:
                self.robot.execute_return(self.base_station_loc, self.base_station_angle,
                                          self.allowed_docking_pos_error, self.allowed_heading_error, database)

            elif self.robot.phase == Phase.DOCKING:
                self.robot.execute_docking()
            
            #update the database with the most recent state
            database.update_data("phase", self.robot.phase)



