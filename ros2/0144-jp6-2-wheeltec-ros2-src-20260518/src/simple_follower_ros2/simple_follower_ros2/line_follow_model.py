#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import cv_bridge
from geometry_msgs.msg import Twist
last_erro=0
tmp_cv = 0
def nothing(s):
    pass
lower_red = (0,70,50)
upper_red = (10,255,255)
lower_blue = (100,70,50)
upper_blue = (125,255,255)
lower_green = (40,70,50)
upper_green = (80,255,255)
lower_yellow = (20,70,50)
upper_yellow = (35,255,255)
lower_balck = (0,0,0)
upper_black = (180,255,40)


Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


class Follower(Node):
    def __init__(self):
        super().__init__('line_follow_model')
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.mat = None
        self.declare_parameter('image_input', '/camera/color/image_raw')
        self.image_input = self.get_parameter('image_input').get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image,
            self.image_input,
            self.image_callback,
            qos)
        self.declare_parameter("target_color", 0)
        self.target_color = self.get_parameter('target_color').get_parameter_value().integer_value
        # print(f"target_color:{self.target_color}")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.twist = Twist()
        self.tmp = 0
        if self.target_color == 0:
            self.targetLower = lower_red
            self.targetUpper = upper_red
            print("Red!")
        elif self.target_color == 1:
            self.targetLower = lower_green
            self.targetUpper = upper_green
            print("Green!")
        elif self.target_color == 2:
            self.targetLower = lower_blue
            self.targetUpper = upper_blue
            print("Bule!")
        elif self.target_color == 3:
            self.targetLower = lower_yellow
            self.targetUpper = upper_yellow
            print("Yellow!")
        elif self.target_color == 4:
            self.targetLower = lower_balck
            self.targetUpper = upper_black
            print("Black!")
        else:
            self.targetLower = lower_red
            self.targetUpper = upper_red
            print("No valid color selected")

    def image_callback(self, msg):
        global last_erro
        global tmp_cv
        #if self.tmp==0:
            #cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            # cv2.createTrackbar(Switch,'Adjust_hsv',0,4,nothing)
            #self.tmp=1
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        kernel = numpy.ones((5,5),numpy.uint8)
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
        # m=cv2.getTrackbarPos(Switch,'Adjust_hsv')

        mask=cv2.inRange(hsv_dilate,self.targetLower,self.targetUpper)
        masked = cv2.bitwise_and(image, image, mask=mask)
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        h, w, d = image.shape
        search_top = h-30
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # 计算mask图像的重心，即几何中心
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
            #cv2.circle(image, (cx-60, cy), 10, (0, 0, 255), -1)
            #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)
            if cv2.circle:
            # 计算图像中心线和目标指示线中心的距离
                erro = cx - w/2-60
                d_erro=erro-last_erro
                self.twist.linear.x = 0.11
                if erro<0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                elif erro>0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                else :
                    self.twist.angular.z = 0.0
                last_erro=erro
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        # cv2.imshow("Adjust_hsv", mask)
        # cv2.waitKey(3)
        #cv2.imshow("Adjust_hsv", mask)
        #print('start windows')
        #cv2.waitKey(3)
def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    while rclpy.ok():
        rclpy.spin_once(follower)
        time.sleep(0.1)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
