import engine.pid_controller as pid_controller

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


class TestGainsNoLimit(unittest.TestCase):
  def test_individual_gains(self): 
    pid_kp = pid_controller.PID(Kp = 3.5, Ki = 0.0, Kd = 0.0, target = 0, 
        sample_time = 0.01, output_limits = (None, None))
    pid_ki = pid_controller.PID(Kp = 0.00, Ki = 10.0, Kd = 0.0, target = 0, 
        sample_time = 0.01, output_limits = (None, None))
    pid_kd = pid_controller.PID(Kp = 0.00, Ki = 0.0, Kd = 0.5, target = 0, 
        sample_time = 0.01, output_limits = (None, None))
    error = 1
    self.assertEqual(1, pid_kp.update(error))
    self.assertEqual(1, pid_ki.update(error))
    self.assertEqual(1, pid_kd.update(error))

  def test_combined_gains(self): 
    pid_kp_ki = pid_controller.PID(Kp = 14.02, Ki = 10.5, Kd = 0.0, target = 0, 
        sample_time = 0.01, output_limits = (None, None))
    pid_all = pid_controller.PID(Kp = 20.05, Ki = 5.0, Kd = 0.8, target = 0, 
        sample_time = 0.01, output_limits = (None, None))
    error = 1 
    self.assertEqual(1, pid_kp_ki.update(error))
    self.assertEqual(1, pid_all.update(error))

class TestNegGainsNoLimit(unittest.TestCase): 
  def neg_gains(self):
    pid_neg_kp = pid_controller.PID(Kp = -7.0, Ki = 0.0, Kd = 0.0, target = 0, 
          sample_time = 0.01, output_limits = (None, None))
    pid_neg_ki = pid_controller.PID(Kp = 10.0, Ki = -5.7, Kd = 0.0, target = 0, 
          sample_time = 0.01, output_limits = (None, None))
    pid_neg_kd = pid_controller.PID(Kp = 10.0, Ki = 5.7, Kd = -2.0, target = 0, 
          sample_time = 0.01, output_limits = (None, None))
    #TODO: assert equal for throwing a negative gains value error 
    # google how to write this test case


#TODO: would need to identify what controllers we have 
# since limits would be different for position, velocity, torque, 
# and heading. Also need to consider why we need this specific limits 
# in the first place. 
class TestGainsWithLimit(unittest.TestCase):
  def pid_with_limits(self):
    pid_neg_kp = pid_controller.PID(Kp = -7.0, Ki = 0.0, Kd = 0.0, target = 0, 
          sample_time = 0.01, output_limits = (None, None))
