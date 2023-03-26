import RPi.GPIO as GPIO
import time

class Breakbeam:
    ACCEPTABLE_TIME = 50 # in seconds
    BEAM_PINS = {17:'50%_1', 22:'50%_2', 23:'100%_1', 24:'100%_2'}

    def __init__(self):
        for BEAM_PIN in Breakbeam.BEAM_PINS.keys():
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(BEAM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(BEAM_PIN, GPIO.BOTH, callback=self.break_beam_callback)
            print(self.timer(BEAM_PIN))
        self.beam_broken = False
        self.blocked_sensors = []
    
    def break_beam_callback(self, channel):
        print(not GPIO.input(channel), channel)
        self.beam_broken = not GPIO.input(channel)
        if not GPIO.input(channel):
            self.timer(channel)
        return self.beam_broken
    
    def timer(self, channel):
        start = time.time()
        while time.time() - start < Breakbeam.ACCEPTABLE_TIME:
            if GPIO.input(channel) == False:
                print('false reading')
                if Breakbeam.BEAM_PINS[channel] in self.blocked_sensors: #it doesnt log duplicate values
                    self.blocked_sensors.remove(Breakbeam.BEAM_PINS[channel])
                break
        else:
            print('Time passed was ', time.time() - start, 'seconds')
            self.blocked_sensors.append(Breakbeam.BEAM_PINS[channel])
            self.check_full()
            return channel
        
    def check_full(self):
        if len(self.blocked_sensors) == 2 and '50%_1' in self.blocked_sensors and '50%_2' in self.blocked_sensors:
            return 'it was the 50 layer'
        
        if len(self.blocked_sensors) == 4:
            return 'its full'

# Allows to exit program and clean up Pi for testing
message = input("Press enter to quit\n\n")
GPIO.cleanup()