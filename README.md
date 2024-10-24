# Sensors

This repository is designed to handle anything regarding to miscellaneous sensors connected to the rover

## Sensor Scripts

### GPS:

#### Scripts: gps.py

#### Published Topics:

/odom/gps

#### Published Messages:

std_msgs/Float32MultiArray

#### Description:

Publishes [latitude, longitude] from the GPS

### IMU:

#### Scripts: imu.py

#### Published Topics:

/movement/Imu

#### Published Messages:

std_msgs/Float64

#### Description:

Publishes the z-axis orientation of the rover, 0 being East increasing CCW

### IMU Calibration:

#### Scripts: imu_calibration.py

#### Description:

Calibrates the IMU. Follow Instructions

### Battery

#### Scripts: battery.py

#### Description:

Prints all the info of the rover. Look at script for info