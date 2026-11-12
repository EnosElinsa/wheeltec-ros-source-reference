"""Validate RTAB-Map base-frame VO and reconstruct VINS body velocity.

The strategy-9 wrapper returns a base-frame pose but does not guarantee a
populated twist. Differences are computed in the odom frame then rotated into
base_footprint. No ground truth is used and timestamps are not restamped.
"""
from copy import deepcopy
import math,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from .core import odometry_contract,body_velocity
from wheeltec_vision_nav.runtime_safety import covariance_valid


class Adapter(Node):
    def __init__(self):
        super().__init__('sim_odom_adapter')
        self.vins=bool(self.declare_parameter('vins',False).value)
        self.vio=bool(self.declare_parameter('vio',False).value)
        self.last=None;self.imu_receipt=None
        self.pub=self.create_publisher(Odometry,'/odom_vo',10)
        self.create_subscription(Odometry,'/sim/odom_raw',self.cb,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/data_raw',self.imu,qos_profile_sensor_data)
    def imu(self,msg):
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if -.05<=age<=.3:self.imu_receipt=time.monotonic()
    def cb(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_footprint':return
        if self.vio and (self.imu_receipt is None or time.monotonic()-self.imu_receipt>.3):
            self.last=None;return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation;t=msg.twist.twist
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if odometry_contract((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),
                (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z),
                msg.pose.covariance,stamp,self.get_clock().now().nanoseconds,
                self.last[0] if self.last else None):return
        out=deepcopy(msg)
        if not self.vins and not covariance_valid(msg.twist.covariance):return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.vins:
            old=self.last; self.last=(stamp,p.x,p.y,yaw)
            if old is None:return
            velocity=body_velocity(old,self.last)
            if velocity is None:return
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.angular.z=velocity
            # Explicit conservative heuristic, not calibrated estimator uncertainty.
            out.twist.covariance=[0.]*36
            for i in range(6):out.twist.covariance[i*7]=.05 if i in (0,1,5) else 1e3
        else:self.last=(stamp,p.x,p.y,yaw)
        self.pub.publish(out)


def main():
    rclpy.init();n=Adapter()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
