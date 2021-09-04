Set-up for Nexus software environment:

Installing virtual environment (venv) & nexus software dependencies:

1. Ensure python3 is downloaded
2. Navigate to the <software> directory
3. Create venv by running:  
   * macOS: <python3 -m venv venv>
   * linux: <python3 -m venv venv>
   * windows: <py -m venv venv>
4. Activate the venv by running:  
   *macOS: <venv/bin/activate>
   *linux: <venv/bin/activate>
   *windows: <venv\Scripts\activate>
5. Run: <pip install -r setup.txt> to install nexus software dependencies.

For IMU lsm9ds1, must be operating on Raspberry Pi to run these commands.

1. Enable I2C using <sudo raspi-config>
2. run <sudo apt-get install python3-dev python3-pip python3-smbus i2c-tools -y>
3. run <sudo i2cdetect -y 1> and make sure that a fair bit displayed on the command line
   (aka a number is being displayed). If nothing is being displayed, recheck the IMU connections.
4. run <sudo pip3 install adafruit-circuitpython-LSM9DS1>
5. make sure you are using python3. other versions of python do not support the lsm9ds1 and busio libraries from
   circuitpython.

For RF Module, must be operating on Raspberry Pi to run these commands.

1. Enable Serial data transmission from RF Module and raspberry pi
   <sudo raspi-config>
2. Go to "interfacing options" and select the "Serial" option. Make the login shell accessible. 