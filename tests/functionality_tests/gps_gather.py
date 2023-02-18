from engine.sensor_module import SensorModule
import csv
import time

sm = SensorModule(write=False)

while True:

    sm.update_gps_data()
    time.sleep(1)

    print(sm.get_measurement((-76.48263089999999, 42.444707799999996)))