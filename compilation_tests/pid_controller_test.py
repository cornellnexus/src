import engine.pid_controller as pid_controller
import unittest

'''
Unit tests for pid_controller.py
'''

"""
Test cases: 
1. Output Limits are none, Kp > 0, other gains are 0 
2. Output Limits are none Ki > 0, other gains are 0
3. Output Limits are none, Kd > 0, other gains are 0 
4. Output Limits are none, Kp, Ki > 0, Kd = 0 
5. Output Limits are none, gains are all > 0 
6. Output Limits are not none (what is expected behavior in this condition?)
  ==> Will anything happen? 
7. Output Limits are none, gains entered are negative 
  ==> Shouldn't be negative gains (need to implement a try-expect statement)
8. Test with sample time? 
9. Test with target?
10. Test error calculation? 
"""
class TestPosPIDController(unittest.TestCase):
  def test_gains_no_lim(self): 
    positive_pid = pid_controller.PID(Kp = 3.5, Ki = 10.0, Kd = 0.5, target = 0, 
        sample_time = 0.01, output_limits = (None, None))

    error = 1
    prev_error = 0
    pid_val = positive_pid.update(error)

    test_proportional = 3.5 * error 
    test_integral = 0 + (10.0 * error * 0.01)
    test_derivative = 0.5 * ((error - prev_error))/0.01
    test_val = test_proportional + test_integral + test_derivative

    self.assertEqual(test_val, pid_val)

  def test_neg_gains_no_limit(self):
    neg_pid = pid_controller.PID(Kp = -7.0, Ki = -5.7, Kd = -2.0, target = 0, 
          sample_time = 0.01, output_limits = (None, None))
    #TODO: assert equal for throwing a negative gains value error 
    # google how to write this test case

  #TODO: would need to identify what controllers we have 
  # since limits would be different for position, velocity, torque, 
  # and heading. Also need to consider why we need this specific limits 
  # in the first place. (integral windup case)
  def test_pid_with_limits(self):
    pid_limit = pid_controller.PID(Kp = 5, Ki = 2, Kd = 1, target = 0, 
          sample_time = 0.01, output_limits = (-10, 10))

if __name__ == '__main__':
    unittest.main()
