#!/usr/bin/env python3
# coding: utf-8

import cv2
import math
import torch
import torchvision
import rclpy
import time
import threading
import subprocess
import sys
import os
import numpy as np
from cv_bridge import CvBridge
#import utils
#print(utils.__file__)
from sensor_msgs.msg import Image
from wheeltec_jetracer.utils import preprocess
from torch2trt import torch2trt
from torch2trt import TRTModule
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wheeltec_jetracer_msg.msg import LaserPosition
from serial.serialutil import SerialException
from vision_msgs.msg import Detection2DArray

class RoadFollowingNode(Node):
    def __init__(self):
        super().__init__('wheeltec_jetracer')
        
        # 参数初始化
        self.declare_parameter('video', '/dev/video0')
        self.declare_parameter('GOAL_X_MIN', -1.0)
        self.declare_parameter('GOAL_X_MAX', 1.0)
        self.declare_parameter('normal_vel', 0.15)
        self.declare_parameter('STEERING_GAIN', 0.35)
        self.declare_parameter('STEERING_BIAS', 0.02)
        self.declare_parameter('turnright_1', 0.0)
        self.declare_parameter('turnright_2', 0.0)
        self.declare_parameter('mec_duration', 0.0)
        self.declare_parameter('parkin_1', 0.0)
        self.declare_parameter('parkin_2', 0.0)
        self.declare_parameter('parkin_3', 0.0)
        self.declare_parameter('parkin_4', 0.0)
        self.declare_parameter('if_akm_yes_or_no', 'yes')
        
        # 获取参数
        self.video = self.get_parameter('video').value
        self.GOAL_X_MIN = self.get_parameter('GOAL_X_MIN').value
        self.GOAL_X_MAX = self.get_parameter('GOAL_X_MAX').value
        self.normal_vel = self.get_parameter('normal_vel').value
        self.STEERING_GAIN = self.get_parameter('STEERING_GAIN').value
        self.STEERING_BIAS = self.get_parameter('STEERING_BIAS').value
        self.turnright_1 = self.get_parameter('turnright_1').value
        self.turnright_2 = self.get_parameter('turnright_2').value
        self.mec_duration = self.get_parameter('mec_duration').value
        self.parkin_1 = self.get_parameter('parkin_1').value
        self.parkin_2 = self.get_parameter('parkin_2').value
        self.parkin_3 = self.get_parameter('parkin_3').value
        self.parkin_4 = self.get_parameter('parkin_4').value
        self.if_akm_yes_or_no = self.get_parameter('if_akm_yes_or_no').value
        
        # 初始化变量
        self.bridge = CvBridge()
        self.model_trt = TRTModule()
        self.autodrive = 0
        self.goal_x = 0.0
        self.side_flag = 0
        self.crossing_flag = 0
        self.crossing_ymax = 0
        self.bus_flag = 0
        self.stop_flag = 0
        self.school_flag = 0
        self.slow_flag = 0
        self.straight_flag = 0
        self.parking_flag = 0
        self.crossing_sign_flag = 0
        self.construction_flag = 0
        self.old_flag = ""
        self.old_boxe_x = -1
        self.minranges = 100.0
        self.min_angleX = 0.0
        #self.video = 0
        self.red_flag = 0
        self.red_light_ymax = 0
        self.red_light_counter = 0
        self.red_light_max_frames = 5
        self.yellow_flag = 0
        self.yellow_light_ymax = 0
        self.yellow_light_counter = 0
        self.yellow_light_max_frames = 5
        self.green_flag = 0
        self.green_light_counter = 0
        self.green_light_max_frames = 5
        self.distance_front = 0
        self.distance_r5 = 0
        self.distance_l45 = 0
        self.distance_l75 = 0
        self.last_play_finish_time = 0
        self.min_interval = 10.0
        self.ii = 0
        self.audio_lock = threading.Lock()
        self._shutdown = False
        
        # 创建发布者
        self.pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        
        # 添加检测话题参数
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('detection_confidence_threshold', 0.5)
        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_conf_thresh = self.get_parameter('detection_confidence_threshold').value

        # 订阅检测结果
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10
        )
        
        self.scandetect = self.create_subscription(
            LaserPosition,
            '/laser_distance',
            self.laser_detect_callback,
            10
        )
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.video)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("please wait a minute",flush=True)
        
        # 初始化模型
        self.init_model()

        # 添加线程锁和退出标志
        self.lock = threading.Lock()
        self._shutdown = False

        # 启动图像采集线程
        self.image_thread = threading.Thread(target=self.image_loop, daemon=True)
        self.image_thread.start()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        print("init ok!----",flush=True)

    def image_loop(self):
        """循环采集图像、发布并推理"""
        while rclpy.ok() and not self._shutdown:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # 发布原始图像
            image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.image_pub.publish(image_msg)
            
            # 推理（原 image_callback 中的处理）
            image = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
            image = preprocess(image).half()
            output = self.model_trt(image).detach().cpu().numpy().flatten()
            raw_goal_x = float(output[0])
            self.goal_x = max(self.GOAL_X_MIN, min(self.GOAL_X_MAX, raw_goal_x))
            
            # 控制循环频率
            time.sleep(0.1)  
    
    def destroy_node(self):
        """确保图像线程正确退出"""
        self._shutdown = True
        if hasattr(self, 'image_thread') and self.image_thread.is_alive():
            self.image_thread.join(timeout=1.0)
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        super().destroy_node()
        
    def init_model(self):
        """初始化神经网络模型"""
        CATEGORIES = ['apex']
        device = torch.device('cuda')
        model = torchvision.models.resnet18(pretrained=False)
        model.fc = torch.nn.Linear(512, 2 * len(CATEGORIES))
        model = model.cuda().eval().half()
        model.load_state_dict(torch.load('/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/model/road_following_model.pth'))
        data = torch.zeros((1, 3, 224, 224)).cuda().half()
        self.model_trt = torch2trt(model, [data], fp16_mode=True)
        torch.save(self.model_trt.state_dict(), '/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/model/road_following_model_trtb.pth')
        self.model_trt.load_state_dict(torch.load('/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/model/road_following_model_trtb.pth'))
        
    def pub_cmd(self, vel, turn):
        """发布控制命令"""
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = turn
        self.pub.publish(msg)
        
    def car_autodrive(self):
        """自动驾驶"""
        turn_z = -self.goal_x * self.STEERING_GAIN + self.STEERING_BIAS
        if turn_z == 0:
            self.pub_cmd(self.normal_vel, turn_z)
        elif abs(self.normal_vel / turn_z) < 0.8:
            self.pub_cmd(self.normal_vel / 2, turn_z / 2)
        else:
            self.pub_cmd(self.normal_vel, turn_z)
            
    def car_autodrive_no_delay(self):
        """自动驾驶（无延迟）"""
        turn_z = -self.goal_x * self.STEERING_GAIN + self.STEERING_BIAS
        self.pub_cmd(self.normal_vel, turn_z)




    def car_turnright(self):
        """右转"""
        old_vel = self.normal_vel
        self.normal_vel = 0.1
        print(self.crossing_ymax,flush=True)
        dell = 0.0
        if self.crossing_ymax > 250:
            dell = (self.crossing_ymax - 250) / 10 * 0.25
        start_time = time.time()
        duration = 3.5
        while time.time() - start_time < duration:
            self.car_autodrive_no_delay()
            time.sleep(0.05)
        self.normal_vel = old_vel
        self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/3.wav")
        if self.if_akm_yes_or_no == "no":
            self.pub_cmd(0.1, 0.0)
            time.sleep(max(0.0,self.turnright_1 - dell))
            self.pub_cmd(0.05, -0.2)
            time.sleep(self.turnright_2)
            self.pub_cmd(0.0, 0.0)
        elif self.if_akm_yes_or_no == "yes":
            self.pub_cmd(0.1, 0.0)
            time.sleep(max(0.0,self.turnright_1 - dell))
            self.pub_cmd(0.05, -0.125)
            time.sleep(self.turnright_2)
            self.pub_cmd(0.0, 0.0)
            
    def car_crossing(self):
        """通过十字路口"""
        old_vel = self.normal_vel
        self.normal_vel = 0.1
        print(self.crossing_ymax,flush=True)
        dell = 0.0
        if self.crossing_ymax > 250:
            dell = (self.crossing_ymax - 250) / 10 * 0.25
        start_time = time.time()
        duration = 5.0
        while time.time() - start_time < duration:
            self.car_autodrive_no_delay()
            time.sleep(0.05)
        self.normal_vel = old_vel
        self.pub_cmd(0.7, 0.0)
        time.sleep(4.0)
        self.pub_cmd(0.0, 0.0)
        
    def car_stop(self):
        """停车"""
        old_vel = self.normal_vel
        self.normal_vel = 0.1
        print(self.crossing_ymax,flush=True)
        dell = 0.0
        if self.crossing_ymax > 250:
            dell = (self.crossing_ymax - 250) / 10 * 0.25
        start_time = time.time()
        duration = 5.0
        while time.time() - start_time < duration:
            self.car_autodrive_no_delay()
            time.sleep(0.05)
        self.normal_vel = old_vel
        self.pub_cmd(0.0, 0.0)
        self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/6.wav")
        time.sleep(5.0)
        
    def car_straight(self):
        """直行"""
        old_vel = self.normal_vel
        self.normal_vel = 0.1
        print(self.crossing_ymax,flush=True)
        dell = 0.0
        if self.crossing_ymax > 250:
            dell = (self.crossing_ymax - 250) / 10 * 0.25
        start_time = time.time()
        duration = 5.0
        while time.time() - start_time < duration:
            self.car_autodrive_no_delay()
            time.sleep(0.05)
        self.normal_vel = old_vel
        self.pub_cmd(0.3, 0.0)
        self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/4.wav")
        time.sleep(1.5)
        
    def car_construction(self):
        """施工路段"""
        self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/8.wav")
        self.pub_cmd(0.0, 0.0)
        time.sleep(1.0)
        
    def car_slow(self):
        """减速"""
        old_vel = self.normal_vel
        self.normal_vel = 0.1
        print(self.crossing_ymax,flush=True)
        dell = 0.0
        if self.crossing_ymax > 250:
            dell = (self.crossing_ymax - 250) / 10 * 0.25
        start_time = time.time()
        duration = 5.0
        while time.time() - start_time < duration:
            self.car_autodrive_no_delay()
            time.sleep(0.05)
        self.normal_vel = old_vel
        self.pub_cmd(0.05, 0.0)
        self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/5.wav")
        time.sleep(4.0)
        self.pub_cmd(0.0, 0.0)
        
    def car_parkin(self):
        """停车入库"""
        if self.if_akm_yes_or_no == "no":
            old_vel = self.normal_vel
            self.normal_vel = 0.1
            start_time = time.time()
            duration = self.parkin_1 + 5.0
            self.crossing_ymax = 0
            while time.time() - start_time < duration:
                self.car_autodrive_no_delay()
                if self.crossing_ymax > 250:
                    duration = duration - 20
                time.sleep(0.05)
            duration = self.mec_duration
            start_time = time.time()
            while time.time() - start_time < duration:
                self.car_autodrive_no_delay()
                time.sleep(0.05)
            self.normal_vel = old_vel
            self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/7.wav")
            self.pub_cmd(-0.1, 0.3)
            time.sleep(self.parkin_2)
            self.pub_cmd(-0.1, 0.0)
            time.sleep(self.parkin_3)
            self.pub_cmd(0.0, 0.0)
            time.sleep(3.0)
            self.pub_cmd(0.1, 0.0)
            time.sleep(self.parkin_3 + 0.2)
            self.pub_cmd(0.1, -0.3)
            time.sleep(self.parkin_4)
        elif self.if_akm_yes_or_no == "yes":
            old_vel = self.normal_vel
            self.normal_vel = 0.1
            start_time = time.time()
            duration = self.parkin_1
            while time.time() - start_time < duration:
                self.car_autodrive_no_delay()
                time.sleep(0.05)
            self.normal_vel = old_vel
            self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/7.wav")
            self.pub_cmd(-0.1, 0.25)
            time.sleep(self.parkin_2)
            self.pub_cmd(-0.1, 0.0)
            time.sleep(self.parkin_3)
            self.pub_cmd(0.0, 0.0)
            time.sleep(3.0)
            self.pub_cmd(0.1, 0.0)
            time.sleep(self.parkin_3)
            self.pub_cmd(0.1, -0.2)
            time.sleep(self.parkin_4)

    def detection_callback(self, msg):
        current_frame_has_red_light = False
        current_frame_has_yellow_light = False
        current_frame_has_green_light = False
        """处理YOLO检测结果，设置标志位"""
        for detection in msg.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            class_id = hypothesis.class_id
            score = hypothesis.score
            bbox = detection.bbox
            x = bbox.center.position.x
            y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y

            if score < self.detection_conf_thresh:
                continue

            # 类别映射（请根据实际标签修改）
            if class_id == 'red_light':
                if score > 0.3 and x < 320:
                    self.red_flag = 1
                    self.red_light_ymax = y
                    self.red_light_counter = 0
                    current_frame_has_red_light = True
            elif class_id == 'yellow_light':
                if score > 0.3 and x < 320:
                    self.yellow_flag = 1
                    self.yellow_light_ymax = y
                    self.yellow_light_counter = 0
                    current_frame_has_yellow_light = True
            elif class_id == 'green_light':
                if score > 0.3 and x < 320:
                    self.green_flag = 1
                    self.green_light_counter = 0
                    current_frame_has_green_light = True
            elif class_id == 'crossing':
                if (size_x>80) and (y>230) and (y<320) and score > 0.5:
                    self.crossing_flag = 1
                    self.crossing_ymax = y
                    print("crossing_flag",y,flush=True)
                elif self.parking_flag==1 and y>250:
                    self.crossing_ymax = y
                elif self.red_flag == 1 and y>250:
                    self.crossing_ymax = y
                elif self.yellow_flag == 1 and y>250:
                    self.crossing_ymax = y
            elif class_id == 'construction':
                if (size_x>200) and (size_x<400) and score > 0.9:
                    self.construction_flag = 1
                    print("construction_flag",flush=True)
                    self.autodrive = 0
            elif class_id == 'turn':
                if (size_y>70) and (size_x>30) and score > 0.5 and not (1.64 < self.distance_front < 1.90) and not (1.65 < self.distance_r5 < 1.91):
                    self.side_flag = 1
                    print("side_flag",flush=True)
            elif class_id == 'bus':
                if (size_y>60) and (size_x>60) and score > 0.6:
                    self.bus_flag = 1
                    print("bus_flag",flush=True)
                    self.autodrive = 0
            elif class_id == 'stop':
                if (size_y>60) and (size_x>60) and score > 0.8:
                    self.stop_flag = 1
                    print("stop_flag",flush=True)
                    self.autodrive = 0
            elif class_id == 'school':
                if (size_y>60) and (size_x>60) and score > 0.6:
                    self.school_flag = 1
                    print("school_flag",flush=True)
                    self.autodrive = 0
            elif class_id == 'slow':
                if (size_y>60) and (size_x>60) and score > 0.6:
                    self.slow_flag = 1
                    print("slow_flag",flush=True)
                    self.autodrive = 0
            elif class_id == 'straight':
                if (size_y>60) and (size_x>60) and score > 0.8:
                    self.straight_flag = 1
                    print("straight_flag",flush=True)
                    self.autodrive = 0
            elif (size_y > 60) or (size_x > 10):
                if ((x<10) or (x+size_x>630)):
                    if class_id == "parking" and score > 0.8 and self.old_flag == class_id:
                        self.parking_flag = 1
                        print("parking_flag",flush=True)
                        self.autodrive = 0
                        self.old_boxe_x = -1
                        self.old_flag = ""
                    elif class_id == "crossing_sign" and score > 0.8 and self.old_flag == class_id:
                        self.crossing_sign_flag = 1
                        print("crossing_sign_flag",flush=True)
                        self.autodrive = 0
                        self.old_boxe_x = -1
                        self.old_flag = ""
                    else:
                        self.old_flag = class_id
                        self.old_boxe_x = -1
                else:
                    if self.old_flag == class_id:
                        self.old_boxe_x = x
                    else:
                        self.old_flag = class_id
                        self.old_boxe_x = -1

        if not current_frame_has_red_light and self.red_flag == 1:
            self.red_light_counter += 1
        
        if self.red_light_counter >= self.red_light_max_frames:
            self.red_flag = 0
        
        if not current_frame_has_yellow_light and self.yellow_flag == 1:
            self.yellow_light_counter += 1
        
        if self.yellow_light_counter >= self.yellow_light_max_frames:
            self.yellow_flag = 0
        
        if not current_frame_has_green_light and self.green_flag == 1:
            self.green_light_counter += 1
        
        if self.green_light_counter >= self.green_light_max_frames:
            self.green_flag = 0

    def play_audio_file(self, audio_file_path):
        """播放音频文件"""
        def _play_audio():
            try:
                result = subprocess.run(['aplay', "-D", "plughw:CARD=Device,DEV=0", audio_file_path], check=True)
                if result.returncode != 0:
                    print(f"aplay播放异常:{result.stderr}")
            except subprocess.TimeoutExpired:
                print(f"音频播放overtime")
            except Exception as e:
                print(f"音频播放异常:{e}")
                
        thread = threading.Thread(target=_play_audio, daemon=True)
        thread.start()
        
    # def side_flag_callback(self, msg):

    #     pass
        
    def image_callback(self):
        # """图像处理回调"""
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # # 发布原始图像
        image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_pub.publish(image_msg)
        
        # # 处理图像
        image1 = frame
        image = cv2.resize(image1, (224, 224), interpolation=cv2.INTER_AREA)
        image = preprocess(image).half()
        output = self.model_trt(image).detach().cpu().numpy().flatten()
        raw_goal_x = float(output[0])
        self.goal_x = max(self.GOAL_X_MIN, min(self.GOAL_X_MAX, raw_goal_x))
        
    # def registerScan(self, scan_data):
    #     """扫描数据回调"""
    #     self.minranges = scan_data.distance
    #     self.min_angleX = scan_data.angleX
    #     if self.min_angleX > 0:
    #         self.min_angleX = 3.1415 - self.min_angleX
    #     else:
    #         self.min_angleX = -(self.min_angleX + 3.1415)
            
    def laser_detect_callback(self, scan_distance):
        """激光检测回调"""
        self.distance_front = scan_distance.distance_front
        self.distance_r5 = scan_distance.distance_r5
        self.distance_l45 = scan_distance.distance_l45
        self.distance_l75 = scan_distance.distance_l75
        
    def control_loop(self):
        """独立控制线程"""
        ii = 0
        rate = 0.05  # 20Hz
        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()

            # --- 障碍物检测 ---
            with self.lock:
                front = self.distance_front
                r5 = self.distance_r5
                l45 = self.distance_l45
                l75 = self.distance_l75

            if -0.5 < self.distance_front < 0.12 or -0.5 < self.distance_r5 < 0.12 or -0.5 < self.distance_l45 < 0.12 or -0.5 < self.distance_l75 < 0.12:
            # if self.minranges < 0.18 and abs(self.min_angleX) < 1.8:
                self.pub_cmd(0.0, 0.0)
                time.sleep(0.1)
                continue
            
            if self.red_flag == 1:
                if 1.92 < self.distance_front < 2.25 and 1.93 < self.distance_r5 < 2.26:
                    print(f"car in stop zone (distance: {self.distance_front}), stopping...",flush=True)
                    self.pub_cmd(0.0, 0.0)
                    self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/1.wav")
                    time.sleep(1.0)
                else:
                    if self.autodrive == 1:
                        self.car_autodrive()
            elif self.yellow_flag == 1:
                if 1.92 < self.distance_front < 2.25 and 1.93 < self.distance_r5 < 2.26:
                    print(f"car in stop zone (distance: {self.distance_front}), stopping...",flush=True)
                    self.pub_cmd(0.0, 0.0)
                    self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2/wheeltec_jetracer/audio/1.wav")
                    time.sleep(1.0)
                else:
                    if self.autodrive == 1:
                        self.car_autodrive()
            elif self.green_flag == 1:
                if 1.92 < self.distance_front < 2.25 and 1.93 < self.distance_r5 < 2.26:
                    self.play_audio_file("/home/wheeltec/wheeltec_ros2/src/wheeltec_jetracer_ros2_ros2/wheeltec_jetracer/audio/2.wav")
                    pass
            if self.crossing_flag == 1:
                if self.side_flag == 0:
                    self.crossing_flag = 0
                elif self.side_flag == 1 or self.side_flag == 2:
                    self.autodrive = 0
                    
            if self.side_flag == 1:
                self.side_flag = 2
                ii = 0
            elif self.side_flag == 2:
                ii = ii + 1
                if ii > 20:
                    self.side_flag = 0
                    ii = 0
                    
            if self.autodrive == 1:
                self.car_autodrive()
            elif self.autodrive == 0:
                self.autodrive = 1
                if (self.side_flag == 1 or self.side_flag == 2) and self.crossing_flag == 1:
                    print("turn right",flush=True)
                    self.car_turnright()
                    self.side_flag = 0
                    self.crossing_flag = 0
                if self.stop_flag == 1:
                    print("parking",flush=True)
                    self.car_stop()
                    self.stop_flag = 0
                elif self.bus_flag == 1:
                    print("bus",flush=True)
                    self.car_slow()
                    self.bus_flag = 0
                elif self.school_flag == 1:
                    print("school",flush=True)
                    self.car_slow()
                    self.school_flag = 0
                elif self.slow_flag == 1:
                    print("slow",flush=True)
                    self.car_slow()
                    self.slow_flag = 0
                elif self.straight_flag == 1:
                    print("straight",flush=True)
                    self.car_straight()
                    self.straight_flag = 0
                elif self.parking_flag == 1:
                    print("parking",flush=True)
                    self.car_parkin()
                    self.parking_flag = 0
                elif self.crossing_sign_flag == 1:
                    print("crossing",flush=True)
                    self.car_slow()
                    self.crossing_sign_flag = 0
                elif self.construction_flag == 1:
                    print("construction",flush=True)
                    self.construction_flag = 0
                    self.car_construction()

            # 控制循环频率
            elapsed = time.time() - loop_start
            sleep_time = rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    node = RoadFollowingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        #rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 停止小车
        node.pub_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
