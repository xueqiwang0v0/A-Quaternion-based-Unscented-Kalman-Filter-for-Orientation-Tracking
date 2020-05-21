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

## Instructions and Tips
1. You will find a set of IMU data and another set of data that gives the corresponding tracking information from the Vicon motion capture system. Download these files and be sure you can load and interpret the file formats.* The files are given as ‘.mat’ files. Make sure you can load these into python first. (Hint - scipy.io.loadmat - but note that this is the only purpose you may use scipy for.)
2. This will return a dictionary form. Please disregard the following keys and corresponding values: ’version’, ‘header’, ‘global’. The keys, ‘cams’, ‘vals’, ‘rots’, and ‘ts’ are the main data you need to use.
3. Note that the biases and scale factors of the IMU sensors are unknown, as well as the registration between the IMU coordinate system and the Vicon global coordinate system. You will have to figure them out.
4. You will write a function that computes orientation only based on gyro data, and another function that computes orientation only based on accelerometer data. You should check that each function works well before you try to integrate them into a single filter. This is important!
5. You will write a filter to process this data and track the orientation of the platform. You can try to use a Kalman filter, EKF or UKF to accomplish this. The paper linked above utilizes a UKF. You will have to optimize over model parameters. You can compare your resulting orientation estimate with the “ground truth” estimate from the Vicon.
