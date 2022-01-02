class DataBase: 

    def __init__(self, phase = 0, state = [0, 0, 0], is_sim = True, time_step =.1, weight = 0, battery = 0,
                 move_dist = .5, acceleration = [0, 0, 0], magnetic_field = [0, 0, 0], gyro_rotation = [0, 0, 0],
                 position_pid = [0, 0, 0], position_noise = 0, heading_pid = [0, 0, 0]):
        """
        Database Parameters:
            phase: 
            state: 
                x_pos: the x position of the robot, where (0,0) is the bottom left corner of the grid with which
                    the Robot's related Mission was initialized
                y_pos: the y position of the robot
                heading: the theta of the robot in radians, where North on the grid is equal to 0.
            time_step: amount of time that passes between each feedback loop cycle, only used if is_sim is True
            position_pid: position PID in format of [proportional factor, integral factor, derivative factor]
            position_noise: the flat amount of noise added to the robot's phase on each localization step
            heading_pid: heading PID in format of [proportional factor, integral factor, derivative factor]           
            move_dist: the distance in meters that the robot moves per time dt
            turn_angle: the angle in radians that the robot turns per time dt regardless of time step 
            weight: the weight of the trash the robot has collected
        """
        self.core_data = {
            "phase" : phase,
            "state" : state,
            "is_sim" : is_sim,
            "time_step" : time_step,
            "weight" : weight, #not detected by sensors yet
            "battery" : battery, #not detected by sensors yet
            "move_dist" : move_dist,
            "acceleration" : acceleration, #not called in main algorithm yet
            "magnetic_field" : magnetic_field, #not called in main algorithm yet
            "gyro_rotation" : gyro_rotation, #not called in main algorithm yet
            "position_pid" : position_pid,
            "position_noise" : position_noise,
            "heading_pid" : heading_pid
        }

    def __str__(self):
        return "phase: " + str(self.core_data["phase"]) + ",\n" + \
               "state [x, y, heading]: " + str(self.core_data["state"]) + ",\n" + \
               "is_sim: " + str(self.core_data["is_sim"]) + ",\n" + \
               "time_step: " + str(self.core_data["time_step"]) + ",\n" + \
               "weight: " + str(self.core_data["weight"]) + ",\n" + \
               "battery: " + str(self.core_data["battery"]) + ",\n" + \
               "move_dist: " + str(self.core_data["move_dist"]) + ",\n" + \
               "acceleration [x, y, z]: " + str(self.core_data["acceleration"]) + ",\n" + \
               "magnetic_field [x, y, z]: " + str(self.core_data["magnetic_field"]) + ",\n" + \
               "gyro_rotation [x, y, z]: " + str(self.core_data["gyro_rotation"]) + ",\n" + \
               "position_pid [proportional factor, integral factor, derivative factor]: " + str(self.core_data["position_pid"]) + ",\n" + \
               "position_noise: " + str(self.core_data["position_noise"]) + ",\n" + \
               "heading_pid [proportional factor, integral factor, derivative factor]: " + str(self.core_data["heading_pid"])



    def get_data(self, name):
        return self.core_data[name]


    def update_data(self, name, x = None, y = None, z = None):
        '''

        Args:
            name: database key string
            x: corresponding value for the key [name];
                if name should have multiple parameters, this is the first value index
            y: if name should have multiple parameters, this is the second value index
            z: if name should have multiple parameters, this is the third value index if applicable

        Example where database is an instance of DataBase:
            database.update_data("phase", 1) sets phase to 1
            database.update_data("state", 3, 1, 20) sets state to [3, 1, 20]
            database.update_data("state", y = 5) sets the second parameter of state's current value,
                so state is now [3, 5, 20]

        '''
        optional_keys = ["state", "acceleration", "magnetic_field", "gyro_rotation", "position_pid", "heading_pid"]
        if name in optional_keys:
            if x != None:
                self.core_data[name] = [x, self.core_data[name][1], self.core_data[name][2]]
            if y != None:
                self.core_data[name] = [self.core_data[name][0], y, self.core_data[name][2]]
            if z != None:
                self.core_data[name] = [self.core_data[name][0], self.core_data[name][1], z]
        else:
            self.core_data[name] = x