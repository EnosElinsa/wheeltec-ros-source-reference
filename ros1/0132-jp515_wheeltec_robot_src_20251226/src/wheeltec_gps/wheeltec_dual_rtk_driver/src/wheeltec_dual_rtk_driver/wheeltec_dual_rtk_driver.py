#!/usr/bin/env python3
# coding: utf-8

import sys
import math
import time
import signal

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Vector3Stamped 
from um982_serial import UM982Serial
#from um982 import UM982Serial

class UM982DriverNode:

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('um982_driver_node', anonymous=False)

        # 获取参数
        port = rospy.get_param('~port', '/dev/wheeltec_gnss')
        baud = rospy.get_param('~baud', '115200')
        # 打开串口
        try:
            self.um982serial = UM982Serial(port, baud)
            rospy.loginfo(f'Serial {port} open successfully!')
        except Exception as e:
            rospy.logerr(f'Serial {port} failed to open: {e}')
            sys.exit(1)

        # 启动串口数据处理线程
        self.um982serial.start()

        # 初始化发布器
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.utm_pub = rospy.Publisher('/gps/utm_pose', Odometry, queue_size=10)
        self.euler_pub = rospy.Publisher('/gps/euler', Vector3Stamped, queue_size=10)
        # 定时发布任务
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self.pub_task)

    def pub_task(self, event):
        # 从 UM982 获取数据
        if self.um982serial.utmpos is None or self.um982serial.fix is None:
            rospy.logwarn_throttle(5.0, "Waiting for GNSS fix...")
            return
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix
        utm_x, utm_y = self.um982serial.utmpos
        vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
        heading, pitch, roll = self.um982serial.orientation
        this_time = rospy.Time.now()

        # Step 1: 发布 NavSatFix
        fix_msg = NavSatFix()
        fix_msg.header.stamp = this_time
        fix_msg.header.frame_id = 'navsat_link'
        fix_msg.latitude = bestpos_lat
        fix_msg.longitude = bestpos_lon
        fix_msg.altitude = bestpos_hgt
        fix_msg.position_covariance[0] = float(bestpos_latstd) ** 2
        fix_msg.position_covariance[4] = float(bestpos_lonstd) ** 2
        fix_msg.position_covariance[8] = float(bestpos_hgtstd) ** 2
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix_msg)

        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = this_time
        euler_msg.header.frame_id = 'euler_link'  # 可以根据需要选择frame
        euler_msg.vector.x = math.radians(roll)   # 以"弧度"发布（ROS标准）
        euler_msg.vector.y = math.radians(pitch)
        euler_msg.vector.z = math.radians(heading)
        self.euler_pub.publish(euler_msg)
        
        # Step 2: 发布 Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = 'earth'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = utm_x
        odom_msg.pose.pose.position.y = utm_y
        odom_msg.pose.pose.position.z = bestpos_hgt

        quaternion = quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(heading))
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = float(bestpos_latstd) ** 2
        odom_msg.pose.covariance[7] = float(bestpos_lonstd) ** 2
        odom_msg.pose.covariance[14] = float(bestpos_hgtstd) ** 2
        odom_msg.pose.covariance[21] = 0.1
        odom_msg.pose.covariance[28] = 0.1
        odom_msg.pose.covariance[35] = 0.1

        odom_msg.twist.twist.linear.x = vel_east
        odom_msg.twist.twist.linear.y = vel_north
        odom_msg.twist.twist.linear.z = vel_ver
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = float(vel_east_std) ** 2
        odom_msg.twist.covariance[7] = float(vel_north_std) ** 2
        odom_msg.twist.covariance[14] = float(vel_ver_std) ** 2

        self.utm_pub.publish(odom_msg)

    def stop(self):
        rospy.loginfo("Stopping gnss driver...")
        self.um982serial.stop()
        self.timer.shutdown()


if __name__ == '__main__':
    driver = UM982DriverNode()

    def signal_handler(sig, frame):
        driver.stop()
        rospy.signal_shutdown("Shutdown by user")

    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()

