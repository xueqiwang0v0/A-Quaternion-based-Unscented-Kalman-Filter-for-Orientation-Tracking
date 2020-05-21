#data files are numbered on the server.
#for exmaple imuRaw1.mat, imuRaw2.mat and so on.
#write a function that takes in an input number (1 through 6)
#reads in the corresponding imu Data, and estimates 
#roll pitch and yaw using an extended kalman filter

import os
import scipy.io as io
import numpy as np
from matplotlib import pyplot as plt
import math
from estimate_rot import *

a = estimate_rot(3)
imu = io.loadmat('imu/imuRaw'+str(3)+'.mat')
imu_val = imu['vals']
ts_imu = imu['ts']
vicon = io.loadmat('vicon/viconRot'+str(3)+'.mat')
rots = vicon['rots']
t = vicon['ts']

vr = np.zeros(rots.shape[2])
vp = np.zeros(rots.shape[2])
vy = np.zeros(rots.shape[2])
for i in range(rots.shape[2]):
    vr[i],vp[i],vy[i] = rot2euler(rots[:,:,i])

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=[8, 8])
ax1.plot(t[0],vr)
ax1.set_title("roll")
ax2.plot(t[0],vp)
ax2.set_title("pitch")
ax3.plot(t[0],vy)
ax3.set_title("yaw")

ax1.plot(ts_imu[0], a[0])
ax2.plot(ts_imu[0], a[1])
ax3.plot(ts_imu[0], a[2])


plt.show()