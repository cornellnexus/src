class DataBase: 

    def init(self):
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
            "phase" : 0, 
            "state" : [0, 0, 0], 
            "is_sim" : True,
            "time_step" : .1, 
            "weight" : 0, #not detected by sensors yet 
            "battery" : 0, #not detected by sensors yet
            "move_dist" : .5, 
            "acceleration" : [0, 0, 0], #not called in main algorithm yet
            "magnetic_field" : [0, 0, 0], #not called in main algorithm yet
            "gyro_rotation" : [0, 0, 0], #not called in main algorithm yet
            "position_pid" : [0, 0, 0],
            "position_noise" : 0,
            "heading_pid" : [0, 0, 0]
        }
        
    def get_data(self): 
        return self.core_data 

    def update_data(self, name, value): 
        self.core_data[name] = value