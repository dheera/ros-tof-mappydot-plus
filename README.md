# ROS driver for MappyDot Plus TOF sensor(s)

This is a ROS node for the MappyDot plus over I2C. It supports reading from multiple MappyDot plus devices.

## Parameters:

* **device** (string) -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** (list of ints) -- the i2c addresses of the MappyDots. Default is [ 8 ].
* **frame** (string) -- name of the frame. Default is "tof".
* **x** (list of floats) -- the x translations of the sensors. Default is [ 0.0 ].
* **y** (list of floats) -- the y translations of the sensors. Default is [ 0.0 ].
* **z** (list of floats) -- the z translations of the sensors. Default is [ 0.0 ].
* **yaw** (list of floats) -- the yaws of the sensors. Default is [ 0.0 ].

## Outputs topics:
* **/ranges** (std\_msgs/Float32MultiArray) -- raw range data from individual sensors
* **/scan** (sensor\_msgs/LaserScan) -- an assembled LaserScan from all the sensor ranges and the sensor locations

# Usage notes

