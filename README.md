# navigation

Install: 
Run: <pip install -r setup.txt> for most of the dependencies. 

For PigPIO (used for bitbanging in rf_module.py), must be operating on a Raspbian OS. 
  1. install wget 
  2. run the following script: 
      wget https://github.com/joan2937/pigpio/archive/master.zip
      unzip master.zip
      cd pigpio-master
      make
      sudo make install
  3. check: http://abyz.me.uk/rpi/pigpio/download.html if any errors persist. 

For IMU lsm9ds1, must be operating on Raspberry Pi to run these commands. 
  1. Enable I2C using <sudo raspi-config> 
  2. run <sudo apt-get install python3-dev python3-pip python3-smbus i2c-tools -y>
  3. run <sudo i2cdetect -y 1> and make sure that a fair bit displayed on the command line
      (aka a number is being displayed). If nothing is being displayed, recheck
      the IMU connections. 
  4. run <sudo pip3 install adafruit-circuitpython-LSM9DS1>
  5. make sure you are using python3. other versions of python do not support
      the lsm9ds1 and busio libraries from circuitpython. 

For RF Module, must be operating on Raspberry Pi to run these commands. 
  1. Enable Serial data transmission from RF Module and raspberry pi 
    <sudo raspi-config>
  2. Go to "interfacing options" and select the "Serial" option. Make the login 
     shell accessible. 