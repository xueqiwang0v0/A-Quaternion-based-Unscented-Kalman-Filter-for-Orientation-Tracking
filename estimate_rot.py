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


def get_sigma_points(P,Q):
    S = np.linalg.cholesky(P+Q)
    W_i = np.sqrt(6) * S
    sigma = np.concatenate((W_i,-W_i), axis=1)
    
    return sigma

# quaternion multiplication
def q_mul(q0,q1):
    a0,b0,c0,d0 = q0
    a1,b1,c1,d1 = q1
    
    a = -b1 * b0 - c1 * c0 - d1 * d0 + a1 * a0
    b = b1 * a0 - c1 * d0 + d1 * c0 + a1 * b0
    c = b1 * d0 + c1 * a0 - d1 * b0 + a1 * c0
    d = -b1 * c0 + c1 * b0 + d1 * a0 + a1 * d0
    
    q_new = np.array([a,b,c,d], dtype=np.float64)
    
    return q_new

def sigma2Y(gyro, dt, sigma, x):
    x = x.reshape(4)
    Y = np.zeros([sigma.shape[0]+1,sigma.shape[1]])
    angle = np.linalg.norm(gyro) * dt
    
    if angle == 0:
        q_delta = np.array([1,0,0,0])
    else:
        axis = gyro * dt / angle
        q_delta = axis2quaternion(angle,axis)
    
    for k in range(sigma.shape[1]):
        s = vector2quaternion(sigma[:,k])
        Y[:,k] = q_mul(x, s)
        Y[:,k] = q_mul(Y[:,k],q_delta).reshape(4)
    
    return Y

def inverse(q):
    inv = q.copy()
    inv[1:] = -inv[1:]
    inv = inv/np.linalg.norm(inv)
    return inv

# compute the mean
def gradient_descent(Y, x):
    q = Y.copy()
   
    for i in range(100):
        e = np.zeros_like(q)
        e_vec = np.zeros([3,6])
        x_inv = inverse(x)
        for k in range(6):
            e[:,k] = q_mul(q[:,k], x_inv).T
            e_vec[:,k] = quaternion2vector(e[:,k])
        
        ave = np.mean(e_vec, axis=1)   # formula 54
        e_ave = vector2quaternion(ave)
        xk_hat = q_mul(e_ave, x)       # formula 55
        
        e_norm = np.linalg.norm(ave)
        # break the loop in advance if reach the desired precision
        if e_norm < 0.01:
            return xk_hat.T, e_vec
        
        x = xk_hat
        
    return xk_hat.T, e_vec

def Y2Z(Y):
    Z = np.zeros(Y.shape)
    g = np.array([0,0,0,1])
    for i in range(Y.shape[1]):
        Z[:,i] = q_mul(Y[:4,i], g) # formula 27
        Z[:,i] = q_mul(Z[:4,i], inverse(Y[:4,i]))
    
    return Z[1:,:]

def axis2quaternion(angle,axis):
    q = np.array([0, axis[0], axis[1], axis[2]])
    q = q * math.sin(angle/2)
    q[0] = math.cos(angle/2)
    # normalize quaternion
    quaternion = q / np.linalg.norm(q)
    
    return quaternion

# 2.2 process model
def vector2quaternion(vec):
    angle = np.linalg.norm(vec)
    
    if angle == 0:
        return np.array([1,0,0,0])
    
    axis = vec / angle
    
    quaternion = axis2quaternion(angle,axis)
    
    return quaternion

def quaternion2vector(q):
    quat = q.copy()
    
    if quat[0] < 0:
        quat = -quat
        
    theta = 2 * math.acos(quat[0])
    
    if theta == 0:
        return np.array([0,0,0])
    
    vec = theta * quat[1:] / np.sqrt(1-quat[0]**2)
    
    return vec

def quaternion2euler(x):
    a,b,c,d = x
    
    # compute roll
    sinr = 2 * (a*b + c*d)
    cosr = 1 - 2 * (b**2 + c**2)
    roll = math.atan2(sinr, cosr)
    
    # compute pitch
    sinp = 2 * (a*c - b*d)
    if abs(sinp) > 1:
        pitch = math.pi / 2
    pitch = math.asin(sinp)
    
    # compute yaw
    siny = 2 * (a*d + b*c)
    cosy = 1 - 2 * (c**2 + d**2)
    yaw = math.atan2(siny, cosy)
    
    return roll, pitch, yaw

def loaddata(data_num):
    imu = io.loadmat('imu/imuRaw'+str(data_num)+'.mat')
    vals = imu['vals']
    imu_t = imu['ts']
    
    return vals, imu_t

# convert from the raw A/D values to physical units
def scale_IMU(vals, imu_t, data_num):
    # parameters
    g = 9.8   # m/s^2
    vref = 3300.0 # mV
    acc_sensitivity = 0.0033 # mV/g
    acc_scale = vref / 1023.0 * acc_sensitivity
    gyro_sensitivity = 3.33 # mV/(degree/s)
    gyro_scale = vref * math.pi / 1023.0 / 180.0 / gyro_sensitivity
    
    # compute bias
    bias = np.mean(vals[:,:5],axis = 1)
    #if data_num == 4:
        #bias[:3] = np.array([505,500,500])
    
    bias = bias.reshape(6,1)
    
    imu = np.zeros([6, imu_t.shape[1]])
    
    # accelerometers scaling
    imu[0:3,:] = (vals[0:3,:] - bias[0:3]) * acc_scale
    
    # gyros scaling
    imu[3:6,:] = (vals[3:6,:] - bias[3:6]) * gyro_scale
    
    temp = imu.copy()
    imu[3,:] = temp[4,:]
    imu[4,:] = temp[5,:]
    imu[5,:] = temp[3,:]
    
    # flip ax, ay
    imu = imu * np.array([1,1,-1,1,1,1]).reshape(-1,1)
    return imu

def compute_dt(imu_t):
    dt = np.zeros_like(imu_t)
    dt[0,0] = 0
    dt[0,1:] = np.diff(imu_t)
    return dt


def estimate_rot(data_num=1):
    filename = os.path.join(os.path.dirname(__file__), "imu/imuRaw" + str(data_num) + ".mat")
    imu = io.loadmat(filename)
    vals = imu['vals']
    imu_t = imu['ts']
    
    z_imu = scale_IMU(vals, imu_t,data_num)
    dt = compute_dt(imu_t)
    
    P = np.eye(3)
    Q = np.diag([100,100,90])  # process noise covariance
    R = np.diag([300,300,200]) # measurement noise covariance
    
    x = np.zeros(4)
    x[0] = 1
    
    euler = np.zeros([3, imu_t.shape[1]])
    
    for i in range(imu_t.shape[1]):
        sigma = get_sigma_points(P,Q)
        Y = sigma2Y(z_imu[3:6,i], dt[0,i], sigma, x)
        xk_hat, e = gradient_descent(Y,x) # compute the mean
        
        # compute the covariance
        W_i = np.copy(e) # formula 63
        cov = np.dot(W_i,W_i.T)/W_i.shape[1] # formula 64
        Z = Y2Z(Y)
        Z_bar = np.mean(Z, axis=1) # formula 48
        P_zz = np.dot((Z-Z_bar.reshape(3,1)), (Z-Z_bar.reshape(3,1)).T) / Z.shape[1] # formula 68
        P_xz = np.dot(W_i, (Z-Z_bar.reshape(3,1)).T) / Z.shape[1] # formula 70
        P_vv = P_zz + R # formula 69
        K_k = np.dot(P_xz, np.linalg.inv(P_vv)) # formula 72
        v_k = z_imu[:3,i] - Z_bar # formula 44
        x_new = q_mul(xk_hat.reshape(4,1), vector2quaternion(np.dot(K_k,v_k))) # formula 74
        x = x_new.copy()
        
        euler[:,i] = quaternion2euler(x_new)
              
        # update the covariance
        P = cov - np.dot(np.dot(K_k, P_vv), K_k.T) # formula 75
    
    roll = euler[0,:]
    pitch = euler[1,:]
    yaw = euler[2,:]

    return roll, pitch, yaw

def rot2euler(R):
    roll=np.arctan2(R[2,1],R[2,2])
    pitch=np.arctan2((-1)*np.float64(R[2,0]),(((R[2,1])**2+(R[2,2])**2)**0.5))
    yaw=np.arctan2(R[1,0],R[0,0])
    
    return roll, pitch, yaw