from time import sleep
import numpy as np

# pid = PID(0.02, 0.01, 0.005, 0.0, 0.01, (-90,90))

""" 
PID Controller that is used to continuously correct errors. 

"""

"""
TODO: 
1. why do we only have a section for heading error? 
2. for calculate_heading_error, rename "input" param to "current heading"
3. calculate error function => define the preconditions for the inputs 
    (floats for heading, coordinate for position/velocity/torque)
4. how should we establish the sampling time? 
5. at what frequency is the update PID controller function being run
6. Need some way to simulate closed feedback loop in regards to PID 
7. might be good to map out current software's controls system architecture
    and see if it models the diagram we initially created. 
"""


class PID:

    """Initializes fields for PID Controller
    Parameters: 
    Kp: Constant for proportional controller 
    Ki: Constant for integral controller 
    Kd: Constant for derivative controller 
    target: Target value we want to reach 
    sample_time: Frequency the PID controller is correcting values
    output_limits: Limits that the PID controller can correct within 

    Fields: 
    prev_error: Value of the error calculated in the previous cycle
    proportional: Proportional controller computation 
    integral: Integral controller computation 
    derivative: Derivative controller computation 
    """
    def __init__(self, Kp = 0.02, Ki = 0.005, Kd = 0.0, target = 0, 
        sample_time = 0.01, output_limits = (None, None)): 
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target
        self.sample_time = sample_time
        self.output_limits = output_limits 
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    """calculate_heading_error(self,input) is the error between the input 
    and the target value.
    """
    def calculate_heading_error(self, input):
        error = abs(self.target - input)
        return error
        
    #call calculate_error before calling update
    # def update(self, error): 
    #     self.proportional = self.Kp * error 
    #     print("PROPORTIONAL: " + str(self.proportional))
    #     self.integral += self.Ki * error * self.sample_time

    #     #max(min(num, max_value), min_value); avoid integral windup
    #     # TODO: COMMENTED THIS INTEGRAL WINDUP LINE OUT - should only occur when output_limits are not none 
    #     #self.integral = max(min(self.integral, output_limit[1]), output_limit[0]) 
    #     print("INTEGRAL: " + str(self.integral))
    #     self.derivative = self.Kd * ((error - self.prev_error) / self.sample_time)
    #     print("DERIVATIVE: " + str(self.derivative))
    #     value = self.proportional + self.integral + self.derivative 
    #     self.prev_error = error 
    #     print("VALUE: " + str(value))
    #     return value
    
    # def get_distance(targ_coods, pred_curr_coords):
    #     return math.sqrt((targ_coords[0]-pred_curr_coords[0])**2 + \
    #     (targ_coords[1]-pred_curr_coords[1])**2)       
    

    def get_sample_time(self):
        return self.sample_time

    def set_target(self, target): 
        self.target = target

    def get_proportional(self):
        return self.proportional
    
    def get_integral(self):
        return self.integral
    
    def get_derivative(self):
        return self.derivative
    
    """Calculates the values of the controllers and updates them accordingly.
    Returns the corrected value based on the error and previous calculations"""
    def update(self, error): 
        self.proportional = self.Kp * error 
        self.integral += self.Ki * error * self.sample_time
        #max(min(num, max_value), min_value); avoid integral windup
        # TODO: COMMENTED THIS INTEGRAL WINDUP LINE OUT - should only occur when output_limits are not none 
        #self.integral = max(min(self.integral, output_limit[1]), output_limit[0]) 
        self.derivative = self.Kd * ((error - self.prev_error) / self.sample_time)
        value = self.proportional + self.integral + self.derivative 
        self.prev_error = error 
        return value

    # TODO: ADD THIS FUNCTION
    # def get_error( )

#TODO: change the output limits (angular velocity)
# control = PID(0.02, 0.005, 0.0, 90, 0.01, (0,90))
# while True:    
#     control.update(20)
#     sleep(control.sample_time)

