import RPi.GPIO as GPIO
import time

# GPIO pin configuration
pin_a = 22  # GPIO 22 (BCM numbering)
pin_b = 27  # GPIO 27 (BCM numbering)

# Constants
DEBOUNCE_TIME = 0.01  # seconds (to filter noise)
PPR = 400  # Pulses per revolution (adjust based on your encoder)

# Global variables
cumulative_pulse_count = 0
last_a_state = GPIO.LOW

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def get_direction(pin_a, pin_b):
    if GPIO.input(pin_a) == GPIO.input(pin_b):
        return 1  # Clockwise
    else:
        return -1  # Counterclockwise

def main():
    setup_gpio()
   
    global last_a_state
    last_a_state = GPIO.input(pin_a)
    last_debounce_time = time.time()
   
    try:
        while True:
            current_a_state = GPIO.input(pin_a)
            current_b_state = GPIO.input(pin_b)
            current_time = time.time()
           
            if current_a_state != last_a_state:
                if current_time - last_debounce_time >= DEBOUNCE_TIME:
                    direction = get_direction(pin_a, pin_b)
                    cumulative_pulse_count += direction
                    last_debounce_time = current_time
                last_a_state = current_a_state
           
            position_degrees = (cumulative_pulse_count * (360.0 / PPR)) % 360.0
            full_rotations = cumulative_pulse_count // PPR
           
            print(f"Pulse Count: {cumulative_pulse_count}")
            print(f"Position: {position_degrees:.2f} degrees (within current revolution)")
            print(f"Full Rotations: {full_rotations}")
           
            time.sleep(0.01)  # Short sleep to reduce CPU usage
   
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up successfully.")

if __name__ == "__main__":
    main()
