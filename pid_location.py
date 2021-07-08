from time import sleep
from engine_rpi import *
from gpiozero import Robot, DigitalInputDevice
from grid import *
import gps

SAMPLETIME = 1

#Call this method in Engine every 5th node or boundary? 
# -> add constructor for current position/target 
#change next target to be 5th node away or next boundary
target = Grid.queue[0]
KP = 0.02 
KD = 0.01
KI = 0.005 

class Encoder(object): 
  def __init__(self, pin):
    self._value = 0 
    encoder = DigitalInputDevice(pin)
    encoder.when_activated = self._increment
    encoder.when_deactivated = self.increment

  def reset(self):
    self.value += 1
  
  @property
  def value(self):
    return self._value

#14,13 connected to left motor's controller 
#12,11 connected to right motor's controller 
# r = Robot((14,13), (12,11))
# e1 = Encoder(15)
# e2 = Encoder(16)

latitude = latMin 
longitude = longMin 
r.value = (latitude, longitude)

lat_prev_error = 0
long_prev_error = 0 
lat_sum_error = 0 
long_sum_error = 0 

while True: 
  #let lat_value = the reading from the GPS/IMU 
  #working together. likewise for long_value
  lat_error = target[0] - lat_value 
  long_error = target[1] - long_value 

  target_lat += (lat_error * KP) + (((lat_error - lat_prev_error)/SAMPLETIME) * KD) + (lat_sum_error * KI)
  target_long += (long_error * KP) + (((long_error - long_prev_error)/SAMPLETIME) * KD) + (long_sum_error * KI)

  #examples i see online have bounds but idk if that is applicable in our case
  # m1_speed = max(min(1, m1_speed), 0)
  # m2_speed = max(min(1, m2_speed), 0)

  #in engine we use the GPS location to move but that's what we're checking?
  #where we are is accurate bc its after kalman filter
  # make sure imu has moved this distance? 
  #based on error, estimate speed you want to move 
  move(target_lat, target_long)

  lat_prev_error = lat_error
  long_prev_error = long_error    
  lat_sum_error += lat_error
  long_sum_error += long_error


