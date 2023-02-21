import numpy as np
from engine.phase import Phase
from electrical.imu import IMU

class Robot_State:
    """
        Variables:
            x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which
                the Robot's related Mission was initialized
            y_pos: the y position of the robot
            heading: the theta of the robot in radians, where North on the grid is equal to 0.
            epsilon: dictates the turning radius of the robot. Lower epsilon results in tighter turning radius.
            max_v: the maximum velocity of the robot
            radius: the radius of the robot
            is_sim: False if the physical robot is being used, True otherwise
            position_kp: the proportional factor of the position PID
            position_ki: the integral factor of the position PID
            position_kd: the derivative factor of the position PID
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            init_phase: the phase which the robot begins at
            heading_kp: the proportional factor of the heading PID
            heading_ki: the integral factor of the heading PID
            heading_kd: the derivative factor of the heading PID
            heading_noise: ?
            time_step: the amount of time that passes between each feedback loop cycle, should only be used if is_sim
                is True
            move_dist: the distance in meters that the robot moves per time dt
            turn_angle: the angle in radians that the robot turns per time dt regardless of time step
            plastic_weight: the weight of the trash the robot has collected
            battery: the battery of the robot
            motor_controller: the motor controller for the Robot
            linear_v: the current linear velocity of the Robot
            angular_v: the current angular velocity of the Robot
    """

    def __init__(self, x_pos, y_pos, heading, epsilon, max_velocity, radius):
        # User defined parameters
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.heading = heading
        self.epsilon = epsilon
        self.max_velocity = max_velocity
        self.radius = radius

        # Defaults
        self.is_sim = True
        self.position_kp = 1
        self.position_ki = 0
        self.position_kd = 0
        self.position_noise = 0
        self.heading_kp = 1
        self.heading_ki = 0
        self.heading_kd = 0
        self.heading_noise = 0
        self.phase = Phase(1)
        self.time_step = 1
        self.move_dist = 0.5
        self.turn_angle = 3
        self.plastic_weight = 0

        self.state = np.array([[self.x_pos], [self.y_pos], [self.heading]])
        self.truthpose = np.transpose(np.array([[self.x_pos], [self.y_pos], [self.heading]]))

        self.battery = 100
        self.imu = IMU()
        self.acceleration = [0, 0, 0]
        self.magnetic_field = [0, 0, 0]
        self.gyro_rotation = [0, 0, 0]
        self.linear_v = 0
        self.angular_v = 0

    # def rpiToGui():
    #     pass

