"""Preserve synthetic exposure stamps; never emulate D435 emitter metadata."""
from array import array
from copy import deepcopy
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .core import ExposureGate
from .depth import millimetres

def stamp_ns(m): return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

class Inputs(Node):
    def __init__(self):
        super().__init__('sim_inputs')
        self.baseline=float(self.declare_parameter('baseline',.095).value)
        self.gyro=float(self.declare_parameter('gyro_noise',.001).value)
        self.accel=float(self.declare_parameter('accel_noise',.01).value)
        self.drop={k:False for k in ('vision','imu','depth')}
        self.gates={name:ExposureGate(('image','info')) for name in ('left','right','depth')}
        self.pubs={}
        topics={'left':('/sim/left/image_rect','/sim/left/camera_info'),
                'right':('/sim/right/image_rect','/sim/right/camera_info'),
                'depth':('/camera/depth/image_rect_raw','/camera/depth/camera_info')}
        self.paired_image=self.create_publisher(Image,'/vision/paired/infra1/image_rect_raw',10)
        self.paired_info=self.create_publisher(CameraInfo,'/vision/paired/infra1/camera_info',10)
        for name,(image,info) in topics.items():
            self.pubs[name]=(self.create_publisher(Image,image,10),self.create_publisher(CameraInfo,info,10))
        for name in ('left','right'):
            self.create_subscription(Image,f'/sim/raw/{name}/image',lambda m,n=name:self.image(n,m),qos_profile_sensor_data)
            self.create_subscription(CameraInfo,f'/sim/raw/{name}/camera_info',lambda m,n=name:self.camera_info(n,m),qos_profile_sensor_data)
        self.create_subscription(Image,'/sim/raw/left/depth_image',lambda m:self.image('depth',m),qos_profile_sensor_data)
        self.imu_pub=self.create_publisher(Imu,'/imu/data_raw',50)
        self.create_subscription(Imu,'/sim/raw/imu',self.imu,qos_profile_sensor_data)
        for name in self.drop:
            self.create_service(SetBool,'/sim/fault/'+name,lambda req,res,n=name:self.fault(n,req,res))
        self.provenance=self.create_publisher(String,'/sim/provenance',1)
        self.create_timer(1.,lambda:self.provenance.publish(String(data=json.dumps(dict(
            source='Gazebo Fortress synthetic camera/IMU and encoder joints',
            timestamp='original Gazebo integer simulation nanoseconds; unmodified',
            synchronization='left RGB/depth same RGBDCamera; right same rate; exact downstream stereo gate',
            depth_units='raw32FC1 metres -> adapted16UC1 millimetres; invalid/nonpositive/overflow zero; original stamp',
            calibration='ideal rectified pinhole, fixed generated TF; no emitter/projector metadata',faults=self.drop)))))
    def fault(self,name,req,res):
        self.drop[name]=req.data
        for gate in self.gates.values():gate.pending.clear()
        res.success=True; res.message=f'{name} drop={req.data}'; return res
    def camera_info(self,name,msg):
        info=deepcopy(msg); info.header.frame_id='sim_'+name+'_optical'
        if name=='right': info.p[3]=-info.p[0]*self.baseline
        self.release(name,self.gates[name].add('info',stamp_ns(info),info))
        if name=='left': self.release('depth',self.gates['depth'].add('info',stamp_ns(info),deepcopy(info)))
    def image(self,name,msg):
        out=deepcopy(msg); out.header.frame_id='sim_'+('left' if name=='depth' else name)+'_optical'
        if name=='depth':
            try:out.data=array('B',millimetres(msg.encoding,msg.width,msg.height,msg.step,msg.is_bigendian,msg.data))
            except (ValueError,TypeError,BufferError):return
            out.encoding='16UC1';out.step=msg.width*2;out.is_bigendian=False
        self.release(name,self.gates[name].add('image',stamp_ns(msg),out))
    def release(self,name,group):
        if group is None or self.drop['depth' if name=='depth' else 'vision']:return
        image,info=group['image'],group['info']
        if image.width!=info.width or image.height!=info.height:return
        self.pubs[name][0].publish(image); self.pubs[name][1].publish(info)
        if name=='left': self.paired_image.publish(image); self.paired_info.publish(info)
    def imu(self,msg):
        if self.drop['imu']:return
        out=deepcopy(msg); out.header.frame_id='sim_imu'; out.orientation_covariance[0]=-1.
        for i in (0,4,8):
            out.angular_velocity_covariance[i]=max(self.gyro**2,1e-8)
            out.linear_acceleration_covariance[i]=max(self.accel**2,1e-6)
        self.imu_pub.publish(out)
def main():
    rclpy.init();node=Inputs()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
