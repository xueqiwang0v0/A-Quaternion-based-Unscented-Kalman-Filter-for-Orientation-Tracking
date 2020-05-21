# A-Quaternion-based-Unscented-Kalman-Filter-for-Orientation-Tracking

In this project, I implement a Kalman filter to track three dimensional orientation.
Given IMU sensor readings from gyroscopes and accelerometers, you will estimate the 
underlying 3D orientation by determining the appropriate model parameters from gro-
und truth data given by a Vicon motion capture system.

## Paper
This project mainly revolves around implementing the algorithm described in this paper:
[A Quaternion-base Unscented Kalman Filter for Orientation Tracking](https://github.com/xueqiwang0v0/A-Quaternion-based-Unscented-Kalman-Filter-for-Orientation-Tracking/blob/master/quaternion_filter.pdf).

## Data
I deal with data from Imu and compare my resulting orientation estimate with the “ground 
truth” estimate from the Vicon.

## Results
![result](https://github.com/xueqiwang0v0/A-Quaternion-based-Unscented-Kalman-Filter-for-Orientation-Tracking/blob/master/result.png)
