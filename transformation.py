import numpy as np
import math

# 2.2 process model
def vector2quaternion(vec):
    omega_norm = np.linalg.norm(vec)
    angle = omega_norm
    
    if angle == 0:
        return np.array([1,0,0,0])
    
    axis = vec / omega_norm
    
    q = np.array([0, axis[0], axis[1], axis[2]])
    q = q * math.sin(angle/2)
    q[0] = math.cos(angle/2)
    # normalize quaternion
    quaternion = q / np.linalg.norm(q)
    
    return quaternion

def quaternion2vector(quat):
    if quat[0] < 0:
        quat[0] = -1 * quat[0]
    else:
        quat = quat
    theta = 2 * math.acos(quat[0])
    vec = theta * quat[1:] / np.sqrt(1-quat[0]**2)
    
    return vec

# extract roll, pitch and yaw values from rotation matrix
def rot2euler(R):
    roll=np.arctan2(R[2,1],R[2,2])
    pitch=np.arctan2((-1)*np.float64(R[2,0]),(((R[2,1])**2+(R[2,2])**2)**0.5))
    yaw=np.arctan2(R[1,0],R[0,0])
    
    return roll, pitch, yaw

def quaternion2euler(x):
    a,b,c,d = x
    
    # compute roll
    sin = 2 * (a*b + c*d)
    cos = 1 - 2 * (b**2 + c**2)
    roll = math.atan2(sin, cos)
    
    # compute pitch
    sin = 2 * (a*c - b*d)
    pitch = math.asin(sin)
    
    # compute yaw
    sin = 2 * (a*d + b*c)
    cos = 1 - 2 * (c**2 + d**2)
    yaw = math.atan2(sin, cos)
    
    return roll, pitch, yaw