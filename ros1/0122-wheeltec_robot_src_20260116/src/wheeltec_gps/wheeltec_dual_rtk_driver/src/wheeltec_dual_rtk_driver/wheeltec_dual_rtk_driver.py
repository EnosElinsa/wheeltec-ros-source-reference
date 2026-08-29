#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import time
import signal

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3Stamped

from um982_serial import UM982Serial
# from um982 import UM982Serial


class UM982DriverNode(object):

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('um982_driver_node', anonymous=False)
        # 获取参数
        port = rospy.get_param('~port', '/dev/wheeltec_gnss')
        baud = rospy.get_param('~baud', 115200)

        # 打开串口
        try:
            self.um982serial = UM982Serial(port, baud)
            rospy.loginfo('Serial %s open successfully!' % port)
        except Exception as e:
            rospy.logerr('Serial %s failed to open: %s' % (port, str(e)))
            sys.exit(1)

        # 启动串口线程
        self.um982serial.start()

        # 发布器
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.utm_pub = rospy.Publisher('/gps/utm_pose', Odometry, queue_size=10)
        self.euler_pub = rospy.Publisher('/gps/euler', Vector3Stamped, queue_size=10)

        # 20Hz 定时器
        self.timer = rospy.Timer(rospy.Duration(0.05), self.pub_task)

    def pub_task(self, event):
        # 从 UM982 读取数据
        # if self.um982serial.fix is None:
        #     rospy.logwarn_throttle(5.0, "Waiting for GNSS fix...")
        # return
        if (self.um982serial.fix is None or
        self.um982serial.orientation is None or
        self.um982serial.vel is None):
             return
        #rospy.loginfo('Serial open successfully!')
        bestpos_hgt, bestpos_lat, bestpos_lon, \
        bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix

        utm_x, utm_y = self.um982serial.utmpos

        vel_east, vel_north, vel_ver, \
        vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel

        heading, pitch, roll = self.um982serial.orientation

        this_time = rospy.Time.now()

        # ---- NavSatFix ----
        fix_msg = NavSatFix()
        fix_msg.header.stamp = this_time
        fix_msg.header.frame_id = 'navsat_link'
        fix_msg.latitude = bestpos_lat
        fix_msg.longitude = bestpos_lon
        fix_msg.altitude = bestpos_hgt

        fix_msg.position_covariance[0] = bestpos_latstd ** 2
        fix_msg.position_covariance[4] = bestpos_lonstd ** 2
        fix_msg.position_covariance[8] = bestpos_hgtstd ** 2
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix_msg)

        # ---- Euler ----
        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = this_time
        euler_msg.header.frame_id = 'euler_link'
        euler_msg.vector.x = math.radians(roll)
        euler_msg.vector.y = math.radians(pitch)
        euler_msg.vector.z = math.radians(heading)
        self.euler_pub.publish(euler_msg)

        # ---- Odometry ----
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = 'earth'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = utm_x
        odom_msg.pose.pose.position.y = utm_y
        odom_msg.pose.pose.position.z = bestpos_hgt

        q = quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(heading)
        )

        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = bestpos_latstd ** 2
        odom_msg.pose.covariance[7] = bestpos_lonstd ** 2
        odom_msg.pose.covariance[14] = bestpos_hgtstd ** 2
        odom_msg.pose.covariance[21] = 0.1
        odom_msg.pose.covariance[28] = 0.1
        odom_msg.pose.covariance[35] = 0.1

        odom_msg.twist.twist.linear.x = vel_east
        odom_msg.twist.twist.linear.y = vel_north
        odom_msg.twist.twist.linear.z = vel_ver

        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = vel_east_std ** 2
        odom_msg.twist.covariance[7] = vel_north_std ** 2
        odom_msg.twist.covariance[14] = vel_ver_std ** 2

        self.utm_pub.publish(odom_msg)

    def stop(self):
        rospy.loginfo('Stopping gnss driver...')
        self.um982serial.stop()
        self.timer.shutdown()


if __name__ == '__main__':
    driver = UM982DriverNode()

    def signal_handler(sig, frame):
        driver.stop()
        rospy.signal_shutdown('Shutdown by user')

    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()
