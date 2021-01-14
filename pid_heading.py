from time import sleep

# pid = PID(0.02, 0.01, 0.005, 0.0, 0.01, (-90,90))

class PID(object):
    def __init__(self, Kp = 0.02, Ki = 0.005, Kd = 0.0, target = 0, sample_time = 0.01, output_limits = (None, None)): 
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def update(self, input): 
      error = self.target- input 
      self.proportional = self.Kp * error 
      print("PROPORTIONAL: " + str(self.proportional))
      self.integral += self.Ki * error * self.sample_time
      #max(min(num, max_value), min_value); avoid integral windup
      self.integral = max(min(self.integral, 90), -90)
      print("INTEGRAL: " + str(self.integral))
      self.derivative = self.Kd * ((error - self.prev_error) / self.sample_time)
      print("DERIVATIVE: " + str(self.derivative))
      value = self.proportional + self.integral + self.derivative 
      self.prev_error = error 
      print("VALUE: " + str(value))
      return value

control = PID(0.02, 0.005, 0.0, 90, 0.01, (-90,90))
while True:    
    control.update(20)
    sleep(control.sample_time)

