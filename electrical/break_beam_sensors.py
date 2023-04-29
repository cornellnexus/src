import RPi.GPIO as GPIO

BEAM_PIN = 17
beam_broken = False

def break_beam_callback(channel):
    if GPIO.input(BEAM_PIN):
        beam_broken = False
        print(beam_broken)
    else:
        beam_broken = True
        print(beam_broken)
    return beam_broken

def check_break_beam():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BEAM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(BEAM_PIN, GPIO.BOTH, callback=break_beam_callback)

    message = input("Press enter to quit\n\n")
    GPIO.cleanup()
    return break_beam_callback

check_break_beam()
