"""One transform source for SDF poses, static TF and VINS IMU calibration."""
import math
import numpy as np

def rotation_rpy(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def transforms(g):
    camera=np.eye(4);camera[:3,:3]=rotation_rpy(g.camera_roll,g.camera_pitch,g.camera_yaw)
    camera[:3,3]=(g.camera_x,g.camera_y,g.camera_z)
    imu=np.eye(4);imu[:3,:3]=rotation_rpy(g.imu_roll,g.imu_pitch,g.imu_yaw);imu[:3,3]=(g.imu_x,g.imu_y,g.imu_z)
    result={'imu':imu}
    optical=np.eye(4);optical[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    for name,sign in (('left',1),('right',-1)):
        offset=np.eye(4);offset[1,3]=sign*g.baseline/2
        result[name+'_physical']=camera@offset;result[name]=camera@offset@optical
    return result

def rpy(rotation):
    pitch=math.asin(max(-1.,min(1.,-float(rotation[2,0]))))
    if abs(math.cos(pitch))<1e-8:raise ValueError('sensor Euler singularity')
    return math.atan2(rotation[2,1],rotation[2,2]),pitch,math.atan2(rotation[1,0],rotation[0,0])
