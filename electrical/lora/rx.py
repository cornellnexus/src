import time
import spidev
import RPi.GPIO as GPIO
from lora import LoRa, ModemConfig  # Import your LoRa library (similar to RH_RF95)

# Pin configuration for RFM95
RFM95_CS = 10
RFM95_RST = 9
RFM95_INT = 25  # GPIO25 (adjust to your setup)

# Frequency configuration (must match transmitter)
RF95_FREQ = 915.0

# Blinky LED on receipt (GPIO13)
LED = 13

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(RFM95_RST, GPIO.OUT)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 500000

# Initialize LoRa object
lora = LoRa(cs_pin=RFM95_CS, int_pin=RFM95_INT, reset_pin=RFM95_RST, spi=spi)

def setup():
    GPIO.output(RFM95_RST, GPIO.HIGH)
    time.sleep(0.1)

    # Manually reset the LoRa module
    GPIO.output(RFM95_RST, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(RFM95_RST, GPIO.HIGH)
    time.sleep(0.01)

    print("Raspberry Pi LoRa RX Test")

    # Initialize LoRa module
    if not lora.init():
        print("LoRa radio init failed")
        while True:
            time.sleep(1)
    print("LoRa radio init OK!")

    # Set frequency
    if not lora.set_frequency(RF95_FREQ):
        print("Set frequency failed")
        while True:
            time.sleep(1)
    print(f"Set Freq to: {RF95_FREQ} MHz")

    # Set transmission power (23 dBm)
    lora.set_tx_power(23, use_pa_boost=True)

def loop():
    while True:
        # Check if data is available to be received
        if lora.received_packet():
            # Blink the LED on receiving data
            GPIO.output(LED, GPIO.HIGH)

            # Read the received message
            message = lora.read_payload()
            rssi = lora.last_rssi()

            print(f"Received: {message}")
            print(f"RSSI: {rssi}")

            # Send a reply message
            reply_message = "And hello back to you"
            lora.send_packet(reply_message.encode('utf-8'))
            print("Sent a reply")

            # Turn off LED after sending reply
            GPIO.output(LED, GPIO.LOW)

        time.sleep(1)

if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        print("Exiting")
        GPIO.cleanup()
