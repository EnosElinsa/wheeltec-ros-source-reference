#!/usr/bin/env python
# coding=utf-8

import rospy
from math import *
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from table_arm.msg import position_color as PositionMsg
from table_arm.msg import color_ik_result as color_ik_result_Msg
from std_msgs.msg import Int8
from std_msgs.msg import String as StringMsg

last_erro=0
def nothing(s):
    pass


class Visual_Camera:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.i=0
        #cv2.namedWindow("window", 1)
        # 订阅usb摄像头
	self.pictureHeight= 480
	self.pictureWidth = 640
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)#订阅图像话题
        # self.image_sub = rospy.Subscriber("cv_bridge_image", Image, self.image_callback)

    def image_callback(self, msg):
        global last_erro
        image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = image0[self.pictureHeight/16:(self.pictureHeight/16*12),self.pictureWidth/16*4:(self.pictureWidth/16*15)]
        image = cv2.resize(image, (640,480), interpolation=cv2.INTER_AREA)#提高帧率
        image1 = self.bridge.cv2_to_imgmsg(image, 'bgr8')

        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node("visual_camera")
visual_camera = Visual_Camera()
rospy.spin()

