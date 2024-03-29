class PID:
    """
    Instances represent a PID Controller that is used to continuously correct errors.

    INSTANCE ATTRIBUTES:
        # Kp: Constant for proportional controller
        # Ki: Constant for integral controller
        # Kd: Constant for derivative controller
        # target: Target value we want to reach
        # sample_time: Frequency the PID controller is correcting values (same frequency as control loop, which is given by time_step)
        # output_limits: Limits that the PID controller can correct within
        # prev_error: Value of the error calculated in the previous loop
        # proportional: Proportional controller command
        # integral: Integral controller command
        # derivative: Derivative controller command
    """

    def __init__(
        self, Kp=1, Ki=0, Kd=0, target=0, sample_time=0.01, output_limits=(None, None)
    ):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.prev_error = 0
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def update(self, error):
        """
        Calculates the values of the controllers and updates them accordingly. Returns the corrected value based on the
        error and previous calculations.

        Parameters:
        ---------
        error: difference between target and current measurement (heading or position)
        """
        self.proportional = self.Kp * error
        # self.integral += self.Ki * error * self.sample_time

        # Avoid integral windup:
        integral_threshold = 40
        if error < integral_threshold:
            # bottom code already exists, no idea what it means
            # if self.output_limits[0] is not None and self.output_limits[1] is not None:
            #     self.integral = max(
            #         min(self.integral, self.output_limits[1]), self.output_limits[0])
            self.integral += self.Ki * error * self.sample_time

        self.derivative = self.Kd * ((error - self.prev_error) / self.sample_time)
        value = self.proportional + self.integral + self.derivative
        self.prev_error = error
        return value

    def reset_integral(self):
        """
        Resets the integral attribute. This is done because integral isn't reset when recalling PID on a separate occasion.
        """
        self.integral = 0
