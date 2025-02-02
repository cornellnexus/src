import threading
import time
import board
import busio
from adafruit_lsm9ds1 import LSM9DS1_I2C

# --------------------------------READ ME BEFORE LOOKING AT CODE-------------------------------
"""
Most of this code came from this website: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/calibration-with-raspberry-pi-using-blinka.
I changed things slightly here and there to so that it might work with our imu.
From some research, for the magnetometer there are 2 types of offset values that we need to keep
in mind: hard-offset and soft-offset. Subtract each x, y, z hard-iron offset from the respective x, y, z
magnetometer readings. Do similar thing with soft-iron offset values.

Similar process done with gyroscope data (except using zero-rate offsets)

site I used to help understand things --> https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration 
"""
# ---------------------------------------------------------------------------------------------


SAMPLE_SIZE = 500


class KeyListener:
    """Object for listening for input in a separate thread"""

    def __init__(self):
        self._input_key = None
        self._listener_thread = None

    def _key_listener(self):
        while True:
            self._input_key = input()

    def start(self):
        """Start Listening"""
        if self._listener_thread is None:
            self._listener_thread = threading.Thread(
                target=self._key_listener, daemon=True
            )
        if not self._listener_thread.is_alive():
            self._listener_thread.start()

    def stop(self):
        """Stop Listening"""
        if self._listener_thread is not None and self._listener_thread.is_alive():
            self._listener_thread.join()

    @property
    def pressed(self):
        "Return whether enter was pressed since last checked" ""
        result = False
        if self._input_key is not None:
            self._input_key = None
            result = True
        return result

    ############################
    # Magnetometer Calibration #
    ############################


def magnetometer_calibrate():
    i2c = busio.I2C(board.SCL, board.SDA)

    magnetometer = LSM9DS1_I2C(i2c)
    key_listener = KeyListener()
    key_listener.start()

    # could maybe automate when we detect that the hard offset values stop changing
    print("Magnetometer Calibration")
    print("Start SLOWLY moving the board in all directions")
    print("When the magnetic Hard Offset values stop")
    print("changing, press ENTER to go to the next step")
    print("Press ENTER to continue...")
    while not key_listener.pressed:
        pass

    mag_x, mag_y, mag_z = magnetometer.magnetic
    min_x = max_x = mag_x
    min_y = max_y = mag_y
    min_z = max_z = mag_z

    while not key_listener.pressed:
        mag_x, mag_y, mag_z = magnetometer.magnetic

        print(
            "Magnetometer: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                mag_x, mag_y, mag_z
            )
        )

        min_x = min(min_x, mag_x)
        min_y = min(min_y, mag_y)
        min_z = min(min_z, mag_z)
        max_x = max(max_x, mag_x)
        max_y = max(max_y, mag_y)
        max_z = max(max_z, mag_z)

        # Get the hard-iron offset values
        HI_offset_x = (max_x + min_x) / 2
        HI_offset_y = (max_y + min_y) / 2
        HI_offset_z = (max_z + min_z) / 2

        # Get the soft-iron offset values
        mag_scale_x = (max_x - min_x) / 2
        mag_scale_y = (max_y - min_y) / 2
        mag_scale_z = (max_z - min_z) / 2

        avg_rad = (mag_scale_x + mag_scale_y + mag_scale_z) / 3.0

        SI_offset_x = avg_rad/(mag_scale_x)
        SI_offset_y = avg_rad/(mag_scale_y)
        SI_offset_z = avg_rad/(mag_scale_z)

        print(
            "Hard-Iron Offset:  X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                HI_offset_x, HI_offset_y, HI_offset_z
            )
        )
        print(
            "Soft-Iron Offset:  X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                SI_offset_x, SI_offset_y, SI_offset_z
            )
        )
        print("")
        time.sleep(0.01)

    hard_off = (HI_offset_x, HI_offset_y, HI_offset_z)
    soft_off = (SI_offset_x, SI_offset_y, SI_offset_z)
    print(
        "Final Hard-Iron: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
            HI_offset_x, HI_offset_y, HI_offset_z
        )
    )
    print(
        "Final Soft-Iron: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
            SI_offset_x, SI_offset_y, SI_offset_z
        )
    )

    return hard_off, soft_off

    #########################
    # Gyroscope Calibration #
    #########################


def gyro_calibrate():
    i2c = busio.I2C(board.SCL, board.SDA)

    gyro_accel = LSM9DS1_I2C(i2c)
    key_listener = KeyListener()
    key_listener.start()

    gyro_x, gyro_y, gyro_z = gyro_accel.gyro
    min_x = max_x = gyro_x
    min_y = max_y = gyro_y
    min_z = max_z = gyro_z

    print("")
    print("")
    print("Gyro Calibration")
    print("Place your gyro on a FLAT stable surface.")
    print("Press ENTER to continue...")

    while not key_listener.pressed:
        pass

    for _ in range(SAMPLE_SIZE):
        gyro_x, gyro_y, gyro_z = gyro_accel.gyro

        print(
            "Gyroscope: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} rad/s".format(
                gyro_x, gyro_y, gyro_z
            )
        )

        min_x = min(min_x, gyro_x)
        min_y = min(min_y, gyro_y)
        min_z = min(min_z, gyro_z)

        max_x = max(max_x, gyro_x)
        max_y = max(max_y, gyro_y)
        max_z = max(max_z, gyro_z)

        noise_x = max_x - min_x
        noise_y = max_y - min_y
        noise_z = max_z - min_z


        print(
            "Zero Rate Offset:  X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} rad/s".format(
                offset_x, offset_y, offset_z
            )
        )
        print(
            "Rad/s Noise:       X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} rad/s".format(
                noise_x, noise_y, noise_z
            )
        )
        print("")

    gyro_calibration = (offset_x, offset_y, offset_z)

    print(
        "Final Zero Rate Offset: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} rad/s".format(
            offset_x, offset_y, offset_z
        )
    )


while True:
    magnetometer_calibrate()