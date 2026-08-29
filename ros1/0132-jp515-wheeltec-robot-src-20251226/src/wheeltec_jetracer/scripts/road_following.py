#!/usr/bin/env python3
# coding: utf-8


import cv2,cv_bridge
import torch
import torchvision
import utils
print(utils.__file__)
from sensor_msgs.msg import Image
from utils import preprocess
import numpy as np
from torch2trt import torch2trt
import torch
from torch2trt import TRTModule
import rospy
from geometry_msgs.msg import Twist
import math
from darknet_ros_msgs.msg import BoundingBoxes
from simple_follower.msg import position as PositionMsg
from wheeltec_jetracer.msg import laser_position
import time
import threading
import subprocess
import sys
import os
from base import MicArrayController
from serial.serialutil import SerialException


# * If the car wobbles left and right,  lower the steering gain
# * If the car misses turns,  raise the steering gain
# * If the car tends right, make the steering bias more negative (in small increments like -0.05)
# * If the car tends left, make the steering bias more postive (in small increments +0.05)
#car.throttle = 0.8


normal_vel = 0.15
STEERING_GAIN = 0.35   
STEERING_BIAS = 0.02
if_akm_yes_or_no = "yes"

autodrive = 0  #autodrive AI自动驾驶
goal_x = 0.0   #goal_x 小车当前视觉下判断出来的朝向点距离中心点的偏移位置
side_flag = 0  #判断向右交通标志
crossing_flag = 0  #判断斑马线
crossing_ymax = 0   #斑马线在图像的ymax位置的值
bus_flag = 0  #判断公交车交通标志
stop_flag = 0  #判断停止交通标志
school_flag = 0  #判断学校交通标志
slow_flag = 0  #判断减速交通标志
straight_flag = 0  #判断直行交通标志
parking_flag = 0  #判断停车交通标志
crossing_sign_flag = 0  #判断斑马线交通标志
construction_flag = 0  #判断路障交通标志
old_flag = ""		#上一帧识别的交通标志名称
old_boxe_x = -1	#上一帧识别的交通标志x坐标 -1则说明上一帧上一帧无有效交通标志
minranges = 100.0   #障碍物识别距离
min_angleX = 0.0   #障碍物识别角度
video = 0
red_flag = 0  # 红灯识别标志
red_light_ymax = 0  # 红灯在图像的ymax位置的值
red_light_counter = 0  # 未识别到红灯的帧数计数器
red_light_max_frames = 20  # 最大未识别帧数阈值
yellow_flag = 0  # 黄灯识别标志
yellow_light_ymax = 0  # 黄灯在图像的ymax位置的值
yellow_light_counter = 0  # 未识别到黄灯的帧数计数器
yellow_light_max_frames = 60  # 最大未识别帧数阈值
green_flag = 0
green_light_counter = 0
green_light_max_frames = 20
distance_front = 0
distance_r5 = 0
last_play_finish_time = 0
min_interval = 1.0  # 最小间隔1秒
audio_lock = threading.Lock()

#部分动作时间设置
turnright_1 = 0.0
turnright_2 = 0.0

mec_duration = 0.0
parkin_1 = 0.0
parkin_2 = 0.0
parkin_3 = 0.0
parkin_4 = 0.0


rospy.init_node("wheeltec_jetracer")
bridge = cv_bridge.CvBridge()
model_trt = TRTModule()
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
video= rospy.get_param('~video') 

#open usb camera
cap = cv2.VideoCapture(video)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("please wait a minute")


def pub_cmd(vel,turn):
	msg = Twist()
	msg.linear.x = vel
	msg.angular.z = turn
	pub.publish(msg)

def car_autodrive():
	global normal_vel
	global goal_x
	global STEERING_GAIN
	global STEERING_BIAS
	turn_z = -goal_x * STEERING_GAIN + STEERING_BIAS
	if turn_z==0:
		pub_cmd(normal_vel , turn_z)
	elif abs(normal_vel/turn_z) < 0.8:
		pub_cmd(normal_vel/2 , turn_z/2)
	else:
		pub_cmd(normal_vel , turn_z)

def car_autodrive_no_delay():
	global normal_vel
	global goal_x
	global STEERING_GAIN
	global STEERING_BIAS
	turn_z = -goal_x * STEERING_GAIN + STEERING_BIAS
	pub_cmd(normal_vel , turn_z)

def car_turnright():
	global if_akm_yes_or_no
	global normal_vel
	global turnright_1
	global turnright_2
	global crossing_ymax
	global distance_front
	global distance_r5
	old_vel = normal_vel
	normal_vel = 0.1
	print(crossing_ymax)
	dell = 0.0
	if crossing_ymax>250:
		dell = (crossing_ymax-250)/10 *0.25
	start_time = time.time()
	duration = 5.0
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/3.wav")
	if if_akm_yes_or_no=="no":
		pub_cmd(0.1,0.0)
		time.sleep(turnright_1-dell)			##因为是根据同时识别到斑马线与转弯标志，再执行转弯动作，根据识别到斑马线的位置适当调整直行时间
		pub_cmd(0.05,-0.2)
		time.sleep(turnright_2)
		pub_cmd(0.0,0.0)
	elif if_akm_yes_or_no=="yes":
		pub_cmd(0.1,0.0)
		time.sleep(turnright_1-dell)
		pub_cmd(0.05,-0.125)
		time.sleep(turnright_2)
		pub_cmd(0.0,0.0)


def car_crossing():
	global normal_vel
	old_vel = normal_vel
	normal_vel = 0.1
	print(crossing_ymax)
	dell = 0.0
	if crossing_ymax>250:
		dell = (crossing_ymax-250)/10 *0.25
	start_time = time.time()
	duration = 5.0
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	pub_cmd(0.7,0.0)
	time.sleep(4.0)
	pub_cmd(0.0,0.0)

def car_stop():
	global normal_vel
	old_vel = normal_vel
	normal_vel = 0.1
	print(crossing_ymax)
	dell = 0.0
	if crossing_ymax>250:
		dell = (crossing_ymax-250)/10 *0.25
	start_time = time.time()
	duration = 5.0
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	pub_cmd(0.0,0.0)
	play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/6.wav")
	time.sleep(5.0)

def car_straight():
	global normal_vel
	old_vel = normal_vel
	normal_vel = 0.1
	print(crossing_ymax)
	dell = 0.0
	if crossing_ymax>250:
		dell = (crossing_ymax-250)/10 *0.25
	start_time = time.time()
	duration = 5.0
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	pub_cmd(0.3,0.0)
	play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/4.wav")
	time.sleep(1.5)

def car_construction():
	play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/8.wav")
	pub_cmd(0.0,0.0)
	time.sleep(1.0)

def car_slow():
	global normal_vel
	old_vel = normal_vel
	normal_vel = 0.1
	print(crossing_ymax)
	dell = 0.0
	if crossing_ymax>250:
		dell = (crossing_ymax-250)/10 *0.25
	start_time = time.time()
	duration = 5.0
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	pub_cmd(0.05,0.0)
	play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/5.wav")
	time.sleep(4.0)
	pub_cmd(0.0,0.0)

def car_parkin():
	global mec_duration
	global parkin_1
	global parkin_2
	global parkin_3
	global parkin_4
	global if_akm_yes_or_no
	global normal_vel
	global crossing_ymax
	if if_akm_yes_or_no=="no":
		old_vel = normal_vel
		normal_vel = 0.1
		start_time = time.time()
		duration = parkin_1+5.0
		crossing_ymax = 0
		while time.time() - start_time < duration:
			car_autodrive_no_delay()
			if crossing_ymax>250:
				duration = duration -20
			time.sleep(0.05)
		duration = mec_duration
		start_time = time.time()
		while time.time() - start_time < duration:
			car_autodrive_no_delay()
			time.sleep(0.05)
		normal_vel = old_vel
		play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/7.wav")
		pub_cmd(-0.1,0.3)
		time.sleep(parkin_2)
		pub_cmd(-0.1,0.0)
		time.sleep(parkin_3)
		pub_cmd(0.0,0.0)
		time.sleep(3.0)
		pub_cmd(0.1,0.0)
		time.sleep(parkin_3+0.2)
		pub_cmd(0.1,-0.3)
		time.sleep(parkin_4)
	elif if_akm_yes_or_no=="yes":
		old_vel = normal_vel
		normal_vel = 0.1
		start_time = time.time()
		duration = parkin_1
		while time.time() - start_time < duration:
			car_autodrive_no_delay()
			time.sleep(0.05)
		normal_vel = old_vel
		play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/7.wav")
		pub_cmd(-0.1,0.25)
		time.sleep(parkin_2)
		pub_cmd(-0.1,0.0)
		time.sleep(parkin_3)
		pub_cmd(0.0,0.0)
		time.sleep(3.0)
		pub_cmd(0.1,0.0)
		time.sleep(parkin_3)
		pub_cmd(0.1,-0.2)
		time.sleep(parkin_4)

def play_audio_file(audio_file_path):
	def _play_audio():
		global last_play_finish_time
		if not audio_lock.acquire(blocking=False):
			print("音频正在播放，跳过本次请求")
			return
		try:
			current_time = time.time()
			wait_time = last_play_finish_time + min_interval - current_time
			if wait_time > 0:
				time.sleep(wait_time)
			subprocess.run(['aplay', '-D', "plughw:CARD=Device,DEV=0", audio_file_path], check=True)
		except Exception as e:
			print(f"音频播放异常:{e}")
		finally:
			last_play_finish_time = time.time()
			audio_lock.release()

	thread = threading.Thread(target=_play_audio, daemon=True)
	thread.start()

def side_flag_callback(msg):
	global side_flag
	global crossing_flag
	global bus_flag
	global stop_flag
	global school_flag
	global slow_flag
	global straight_flag
	global parking_flag
	global crossing_sign_flag
	global construction_flag
	global autodrive
	global old_flag
	global old_boxe_x
	global crossing_ymax
	global red_flag
	global red_light_ymax
	global red_light_counter
	global yellow_flag
	global yellow_light_ymax
	global yellow_light_counter
	global green_flag
	global green_light_counter
	global distance_front
	global distance_r5
	
	current_frame_has_red_light = False
	current_frame_has_yellow_light = False
	current_frame_has_green_light = False

	for boxes in msg.bounding_boxes:
		if boxes.Class == "red_light":
			print(f"Found red_light: probability={boxes.probability}")
			if boxes.probability > 0.3 and boxes.xmin < 320:
				red_flag = 1
				red_light_ymax = boxes.ymax
				red_light_counter = 0
				current_frame_has_red_light = True
		elif boxes.Class == "yellow_light":
			print(f"Found yellow_light: probability={boxes.probability}")
			if boxes.probability > 0.3 and boxes.xmin < 320:
				yellow_flag = 1
				yellow_light_ymax = boxes.ymax
				yellow_light_counter = 0
				current_frame_has_yellow_light = True
		elif boxes.Class == "green_light":
			print(f"Found green_light: probability={boxes.probability}")
			if boxes.probability > 0.3 and boxes.xmin < 320:
				green_flag = 1
				green_light_counter = 0
				current_frame_has_green_light = True
		elif boxes.Class == "crossing":
			if ((boxes.xmax-boxes.xmin)>80) and (boxes.ymax>250) and (boxes.ymax<320) and boxes.probability > 0.5:
				crossing_flag = 1
				crossing_ymax = boxes.ymax
				print("crossing_flag%d",boxes.ymax)
				#autodrive = 0
			elif parking_flag==1 and boxes.ymax>250:
				crossing_ymax = boxes.ymax
			elif red_flag == 1 and boxes.ymax>250:
				crossing_ymax = boxes.ymax
			elif yellow_flag == 1 and boxes.ymax>250:
				crossing_ymax = boxes.ymax
		elif boxes.Class == "construction":
			if ((boxes.xmax-boxes.xmin)>200) and ((boxes.xmax-boxes.xmin)<400) and boxes.probability > 0.4:
				construction_flag = 1
				print("construction_flag")
				autodrive = 0
		elif boxes.Class == "turn":
			if ((boxes.ymax - boxes.ymin)>70) and ((boxes.xmax - boxes.xmin)>30) and boxes.probability > 0.5 and not (1.64 < distance_front < 1.90) and not (1.65 < distance_r5 < 1.91):
			#if ((boxes.ymax - boxes.ymin)>70) and ((boxes.xmax - boxes.xmin)>30) and boxes.probability > 0.5:
				side_flag = 1
				print("side_flag")
				#autodrive = 0
		elif boxes.Class == "bus":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>60) and boxes.probability > 0.6:
				bus_flag = 1
				print("bus_flag")
				autodrive = 0
		elif boxes.Class == "stop":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>60) and boxes.probability > 0.8:
				stop_flag = 1
				print("stop_flag")
				autodrive = 0
		elif boxes.Class == "school":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>60) and boxes.probability > 0.6:
				school_flag = 1
				print("school_flag")
				autodrive = 0
		elif boxes.Class == "slow":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>60) and boxes.probability > 0.6:
				slow_flag = 1
				print("slow_flag")
				autodrive = 0
		elif boxes.Class == "straight":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>70) and boxes.probability > 0.95:
				straight_flag = 1
				print("straight_flag")
				autodrive = 0
		elif (boxes.ymax - boxes.ymin > 60) or (boxes.xmax - boxes.xmin > 10):
			#当交通标志大小达到一定程度时(> 50/> 30)，再判断交通标志是否处于最左侧位置或者最右侧位置，同时判断相对上一帧old_boxe_x来说该标志是否是从中间朝两边位移
			if ((boxes.xmin<10) or (boxes.xmax>630)):
				#print(f"FLAG:::old_boxe_x: {old_boxe_x} boxes.xmin: {boxes.xmin} boxes.ymin: {boxes.ymin} boxes.ymax: {boxes.ymax} boxes.probability: {boxes.probability}")
				if boxes.Class == "parking" and boxes.probability > 0.95 and old_flag == boxes.Class:
					parking_flag = 1
					print("parking_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "crossing_sign" and boxes.probability > 0.8 and old_flag == boxes.Class:
					crossing_sign_flag = 1
					print("crossing_sign_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				else:
					old_flag = boxes.Class
					old_boxe_x = -1
			else:
				if old_flag == boxes.Class:
					old_boxe_x = boxes.xmin
				else:
					old_flag = boxes.Class
					old_boxe_x = -1

                
    
    	#连续20帧没有识别到红灯小车继续并将red_flag置0
	if not current_frame_has_red_light and red_flag == 1:
		red_light_counter += 1
		#print(f"Red light not detected, counter: {red_light_counter}")
        
	if red_light_counter >= red_light_max_frames:
		#print("Red light not detected for 20 frames, resuming...")
		red_flag = 0

	#连续30帧没有识别到黄灯小车继续并将yellow_flag置0
	if not current_frame_has_yellow_light and yellow_flag == 1:
		yellow_light_counter += 1
		#print(f"Yellow light not detected, counter: {yellow_light_counter}")
        
	if yellow_light_counter >= yellow_light_max_frames:
		#print("Yellow light not detected for 60 frames, resuming...")
		yellow_flag = 0
		
	if not current_frame_has_green_light and green_flag == 1:
		green_light_counter += 1
		
	if green_light_counter >= green_light_max_frames:
		green_flag = 0


def image_callback():
	global bridge
	global goal_x
	global cap
	global image_pub
	rate = rospy.Rate(10)
	
	goal_x_min = rospy.get_param('~GOAL_X_MIN')
	goal_x_max = rospy.get_param('~GOAL_X_MAX')
	#frame_count = 0
	#print_interval = 5
	
	while not rospy.is_shutdown():
		ret , frame = cap.read()
		if not ret:
			break
		#frame_count += 1
		
		image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
		image_pub.publish(image_msg)
		image1 = frame #bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image1, (224,224), interpolation=cv2.INTER_AREA)
		image = preprocess(image).half()
		output = model_trt(image).detach().cpu().numpy().flatten()
		goal_x = float(output[0])  #采集图片给模型，输出当前小车朝向点距离图像中心点的偏移位置
		
		raw_goal_x = float(output[0])
		goal_x = max(goal_x_min, min(goal_x_max, raw_goal_x))
		#if frame_count % print_interval == 0:
			#rospy.loginfo(f"goal_x: {goal_x:.3f}")
		#rate.sleep()

def registerScan(scan_data):
	global minranges
	global min_angleX
	minranges = scan_data.distance
	min_angleX = scan_data.angleX
	if min_angleX>0:
		min_angleX=3.1415-min_angleX;
	else:
		min_angleX=-(min_angleX+3.1415);
		
def laser_detect_callback(scan_distance):
	global distance_front
	global distance_r5
	
	distance_front = scan_distance.distance_front
	distance_r5 = scan_distance.distance_r5
	
	#print("distance_front" + str(distance_front))
	#print("distance_r5" + str(distance_r5))
	#print("--------------------------------------")
	
def run():
	global side_flag
	global crossing_flag
	global crossing_ymax
	global bus_flag
	global stop_flag
	global school_flag
	global slow_flag
	global straight_flag
	global parking_flag
	global crossing_sign_flag
	global construction_flag
	global stop_x
	global stop_y
	global autodrive
	global minranges
	global min_angleX
	global normal_vel
	global red_flag
	global red_light_counter
	global yellow_flag
	global yellow_light_counter
	global green_flag
	global green_light_counter
	global if_akm_yes_or_no
	global distance_front
	global distance_r5
	rate = rospy.Rate(20)
	time.sleep(10.0)
	ii = 0
	while not rospy.is_shutdown():
		if minranges<0.18 and abs(min_angleX)<1.8:        #障碍物 <0.18m ,the car stop
			pub_cmd(0.0,0.0)
			time.sleep(0.1)
			continue
		if red_flag == 1:
			if 1.92 < distance_front < 2.25 and 1.93 < distance_r5 < 2.26:
				print(f"car in stop zone (distance: {distance_front}), stopping...")
				pub_cmd(0.0, 0.0)
				play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/1.wav")
				time.sleep(1.0)
			else:
				if autodrive == 1:
					car_autodrive()
		elif yellow_flag == 1:
			if 1.92 < distance_front < 2.25 and 1.93 < distance_r5 < 2.26:
				print(f"car in stop zone (distance: {distance_front}), stopping...")
				pub_cmd(0.0, 0.0)
				play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/1.wav")
				time.sleep(1.0)
			else:
				if autodrive == 1:
					car_autodrive()
		elif green_flag == 1:
			if 1.92 < distance_front < 2.25 and 1.93 < distance_r5 < 2.26:
				play_audio_file("/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/audio/2.wav")
		#else:
			#if autodrive == 1:
				#car_autodrive()
		if crossing_flag == 1:
			if side_flag == 0:
				crossing_flag = 0
			elif side_flag == 1 or side_flag == 2:
				autodrive = 0		#crossing_flag斑马线标志识别到时，检查是否识别到side_flag，识别到则关闭自动驾驶，进入转弯判断条件中
		if side_flag == 1:
			side_flag = 2
			ii = 0
		elif side_flag == 2:
			ii = ii + 1
			if ii >20:
				side_flag = 0		#side_flag计数20次内，没有识别到转向标志时，转向标志side_flag置0
				ii = 0
		if autodrive == 1:
			car_autodrive()
		elif autodrive == 0:
			autodrive = 1
			if (side_flag == 1 or side_flag == 2) and crossing_flag == 1:
				print("turn right")
				car_turnright()
				side_flag = 0
				crossing_flag = 0
			if stop_flag == 1:
				print("parking")
				car_stop()
				stop_flag = 0
			elif bus_flag == 1:
				print("bus")
				car_slow()
				bus_flag = 0
			elif school_flag == 1:
				print("school")
				car_slow()
				school_flag = 0
			elif slow_flag == 1:
				print("slow")
				car_slow()
				slow_flag = 0
			elif straight_flag == 1:
				print("straight")
				car_straight()
				straight_flag = 0
			elif parking_flag == 1:
				print("parking")
				car_parkin()
				parking_flag = 0
			elif crossing_sign_flag == 1:
				print("crossing")
				car_slow()
				crossing_sign_flag = 0
			elif construction_flag == 1:
				print("construction")
				construction_flag = 0
				car_construction()
		rate.sleep()

def main():
	global model_trt
	global normal_vel
	global STEERING_GAIN
	global STEERING_BIAS
	global mec_duration
	global turnright_1
	global turnright_2
	global parkin_1
	global parkin_2
	global parkin_3
	global parkin_4
	global if_akm_yes_or_no
	CATEGORIES = ['apex']
	device = torch.device('cuda')
	model = torchvision.models.resnet18(pretrained=False)
	model.fc = torch.nn.Linear(512, 2 * len(CATEGORIES))
	model = model.cuda().eval().half()
	model.load_state_dict(torch.load('/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/model/road_following_model.pth'))
	data = torch.zeros((1, 3, 224, 224)).cuda().half()
	model_trt = torch2trt(model, [data], fp16_mode=True)
	torch.save(model_trt.state_dict(), '/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/model/road_following_model_trtb.pth')
	model_trt.load_state_dict(torch.load('/home/wheeltec/wheeltec_robot/src/wheeltec_jetracer/model/road_following_model_trtb.pth'))
	print("init ok!----")
	side_flag_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, side_flag_callback)
	scanSubscriber = rospy.Subscriber('object_tracker/current_position', PositionMsg, registerScan)
	scandetect = rospy.Subscriber("/laser_distance", laser_position, laser_detect_callback)
	#image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

	normal_vel= rospy.get_param('~normal_vel') 
	STEERING_GAIN= rospy.get_param('~STEERING_GAIN') 
	STEERING_BIAS= rospy.get_param('~STEERING_BIAS') 
	turnright_1= rospy.get_param('~turnright_1') 
	turnright_2= rospy.get_param('~turnright_2') 
	mec_duration= rospy.get_param('~mec_duration') 
	parkin_1= rospy.get_param('~parkin_1') 
	parkin_2= rospy.get_param('~parkin_2') 
	parkin_3= rospy.get_param('~parkin_3') 
	parkin_4= rospy.get_param('~parkin_4') 
	if_akm_yes_or_no= rospy.get_param('~if_akm_yes_or_no') 
	if if_akm_yes_or_no=="no":
		print("this car is minimec!!")
	elif if_akm_yes_or_no=="yes":
		print("this car is miniakm!!")


	thread = threading.Thread(target=run)
	thread.start()
	thread2 = threading.Thread(target=image_callback)
	thread2.start()
	rospy.spin()


if __name__ == '__main__':
	try:
		main()
		rospy.spin()
	except rospy.ROSInterruptException:
		pub_cmd(0.0,0.0)
