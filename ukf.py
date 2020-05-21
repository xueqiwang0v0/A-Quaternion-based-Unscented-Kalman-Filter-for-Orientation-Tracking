import numpy as np
import math
from transformation import *

def get_sigma_points(P,Q):
    S = np.linalg.cholesky(P+Q)
    n = S.shape[0]
    W_i = np.sqrt(2*n) * S
    sigma = np.concatenate((W_i,-W_i), axis=1)
    
    return sigma

# quaternion multiplication
def q_mul(q0,q1):
    a0,b0,c0,d0 = q0
    a1,b1,c1,d1 = q1
    
    a = -b1 * b0 - c1 * c0 - d1 * d0 + a1 * a0
    b = b1 * a0 + c1 * d0 - d1 * c0 + a1 * b0
    c = -b1 * d0 + c1 * a0 + d1 * b0 + a1 * c0
    d = b1 * c0 - c1 * b0 + d1 * a0 + a1 * d0
    
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
        q_delta = np.array([0, axis[0], axis[1], axis[2]])
        q_delta = q_delta * math.sin(angle/2)
        q_delta[0] = math.cos(angle/2)
        q_delta = q_delta / np.linalg.norm(q_delta)
    
    for k in range(sigma.shape[1]):
        s = vector2quaternion(sigma[:,k])
        Y[:,k] = q_mul(x, s)
        Y[:,k] = q_mul(Y[:,k],q_delta)
    
    return Y

def inverse(q):
    return np.array([q[0],-q[1],-q[2],-q[3]]) / np.linalg.norm(q)

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
        
        e_norm = np.linalg.norm(e_ave)
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