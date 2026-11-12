"""Wheel odometry from simulated wheel joints; it never reads Gazebo ground truth."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from .core import EncoderOdometry

class WheelOdom(Node):
    def __init__(self):
        super().__init__('sim_wheel_odom')
        self.r=float(self.declare_parameter('wheel_radius',.065).value)
        self.track=float(self.declare_parameter('track',.30).value)
        self.encoder=EncoderOdometry(self.r,self.track)
        self.pub=self.create_publisher(Odometry,'/wheel/odom',10)
        self.create_subscription(JointState,'/joint_states',self.cb,20)
    def cb(self,msg):
        names={n:i for i,n in enumerate(msg.name)}
        try: l=msg.position[names['rear_left_joint']]; rr=msg.position[names['rear_right_joint']]
        except (KeyError,IndexError): return
        stamp=msg.header.stamp; result=self.encoder.update(stamp.sec*1_000_000_000+stamp.nanosec,l,rr)
        if result is None:return
        x,y,yaw,v,wz=result
        o=Odometry(); o.header.stamp=stamp; o.header.frame_id='wheel_odom'; o.child_frame_id='base_footprint'; o.pose.pose.position.x=x; o.pose.pose.position.y=y
        o.pose.pose.orientation=Quaternion(z=math.sin(yaw/2),w=math.cos(yaw/2)); o.twist.twist.linear.x=v; o.twist.twist.angular.z=wz
        for i in range(6):
            o.pose.covariance[i*7]=.01 if i in (0,1,5) else 1e3
            o.twist.covariance[i*7]=.0025 if i in (0,1) else .01 if i==5 else 1e3
        self.pub.publish(o)
def main():
    rclpy.init(); n=WheelOdom()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
