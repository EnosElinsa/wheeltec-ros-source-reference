#!/usr/bin/env python3
# coding: utf-8


import cv2,cv_bridge
import torch
import torchvision
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
import time
import threading


# * If the car wobbles left and right,  lower the steering gain
# * If the car misses turns,  raise the steering gain
# * If the car tends right, make the steering bias more negative (in small increments like -0.05)
# * If the car tends left, make the steering bias more postive (in small increments +0.05)
#car.throttle = 0.8


normal_vel = 0.15
STEERING_GAIN = 0.35   
#转向增益 参数推荐设置
#for miniakm:(normal_vel,STEERING_GAIN)=(0.1,0.35)/(0.15,0.4)/(0.2,0.5)-->
#for minimec:(normal_vel,STEERING_GAIN)=(0.1,0.6)/(0.15,0.8)/(0.2,1.1)--> 
STEERING_BIAS = 0.03
if_akm_yes_or_no = "yes"

autodrive = 0  #autodrive AI自动驾驶
goal_x = 0.0   #goal_x 小车当前视觉下判断出来的朝向点距离中心点的偏移位置
side_flag = 0  #判断向右交通标志
crossing_flag = 0  #判断斑马线
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


bridge = cv_bridge.CvBridge()
model_trt = TRTModule()
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)

#open usb camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)




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
		pub_cmd(normal_vel/3 , turn_z/3)
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
	old_vel = normal_vel
	normal_vel = 0.1
	start_time = time.time()
	duration = 5
	while time.time() - start_time < duration:
		car_autodrive_no_delay()
		time.sleep(0.05)
	normal_vel = old_vel
	if if_akm_yes_or_no=="no":
		pub_cmd(0.1,0.0)
		time.sleep(2.5)
		pub_cmd(0.05,-0.2)
		time.sleep(9.0)
		pub_cmd(0.0,0.0)
	elif if_akm_yes_or_no=="yes":
		pub_cmd(0.1,0.0)
		time.sleep(2.5)
		pub_cmd(0.1,-0.25)
		time.sleep(4.8)
		pub_cmd(0.0,0.0)

def car_crossing():
	pub_cmd(0.1,0.0)
	time.sleep(3.5)
	pub_cmd(0.7,0.0)
	time.sleep(4.0)
	pub_cmd(0.0,0.0)

def car_stop():
	pub_cmd(0.1,0.0)
	time.sleep(2.5)
	pub_cmd(0.0,0.0)
	time.sleep(5.0)

def car_straight():
	pub_cmd(0.1,0.0)
	time.sleep(2.5)
	pub_cmd(0.2,0.0)
	time.sleep(2.0)

def car_construction():
	pub_cmd(0.0,0.0)
	time.sleep(1.0)

def car_slow():
	pub_cmd(0.1,0.0)
	time.sleep(2.5)
	pub_cmd(0.05,0.0)
	time.sleep(4.0)
	pub_cmd(0.0,0.0)

def car_parkin():
	global if_akm_yes_or_no
	if if_akm_yes_or_no=="no":
		pub_cmd(0.1,0.0)
		time.sleep(4.1)
		pub_cmd(-0.1,0.3)
		time.sleep(5.7)
		pub_cmd(-0.1,0.0)
		time.sleep(2.2)
		pub_cmd(0.0,0.0)
		time.sleep(3.0)
		pub_cmd(0.1,0.0)
		time.sleep(2.7)
		pub_cmd(0.05,-0.15)
		time.sleep(10.0)
	elif if_akm_yes_or_no=="yes":
		pub_cmd(0.1,0.0)
		time.sleep(5.1)
		pub_cmd(-0.1,0.2)
		time.sleep(5.8)
		pub_cmd(-0.1,0.0)
		time.sleep(4.2)
		pub_cmd(0.0,0.0)
		time.sleep(3.0)
		pub_cmd(0.1,0.0)
		time.sleep(5.3)
		pub_cmd(0.1,-0.2)
		time.sleep(5.3)


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
	for boxes in msg.bounding_boxes:
		if boxes.Class == "crossing":
			if ((boxes.xmax-boxes.xmin)>100) and (boxes.ymax>240) and (boxes.ymax<260) and boxes.probability > 0.4:
				crossing_flag = 1
				print("crossing_flag")
				#autodrive = 0
		elif boxes.Class == "construction":
			if ((boxes.xmax-boxes.xmin)>200) and ((boxes.xmax-boxes.xmin)<400) and boxes.probability > 0.4:
				construction_flag = 1
				print("construction_flag")
				autodrive = 0
		elif boxes.Class == "turn":
			if ((boxes.ymax - boxes.ymin)>60) and ((boxes.xmax - boxes.xmin)>30) and boxes.probability > 0.3:
				side_flag = 1
				print("side_flag")
				#autodrive = 0
		elif (boxes.ymax - boxes.ymin > 50) or (boxes.xmax - boxes.xmin > 30):
			#当交通标志大小达到一定程度时(> 50/> 30)，再判断交通标志是否处于最左侧位置或者最右侧位置，同时判断相对上一帧old_boxe_x来说该标志是否是从中间朝两边位移
			if ((boxes.xmin<10 and (old_boxe_x-boxes.xmin)>5) or (boxes.xmax>630 and (old_boxe_x-boxes.xmin)<-5)) and old_boxe_x>0 and abs(old_boxe_x-boxes.xmin)<320:
				#print(f"FLAG:::old_boxe_x: {old_boxe_x} boxes.xmin: {boxes.xmin} boxes.ymin: {boxes.ymin} boxes.ymax: {boxes.ymax} boxes.probability: {boxes.probability}")
				if boxes.Class == "bus" and boxes.probability > 0.6 and old_flag == boxes.Class:
					bus_flag = 1
					print("bus_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "stop" and boxes.probability > 0.8 and old_flag == boxes.Class:
					stop_flag = 1
					print("stop_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "school" and boxes.probability > 0.8 and old_flag == boxes.Class:
					school_flag = 1
					print("school_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "slow" and boxes.probability > 0.8 and old_flag == boxes.Class:
					slow_flag = 1
					print("slow_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "straight" and boxes.probability > 0.8 and old_flag == boxes.Class:
					straight_flag = 1
					print("straight_flag")
					autodrive = 0
					old_boxe_x = -1
					old_flag = ""
				elif boxes.Class == "parking" and boxes.probability > 0.8 and old_flag == boxes.Class:
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



def image_callback():
	global bridge
	global goal_x
	global cap
	global image_pub
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ret , frame = cap.read()
		if not ret:
			break
		image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
		image_pub.publish(image_msg)
		image1 = frame #bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image1, (224,224), interpolation=cv2.INTER_AREA)
		image = preprocess(image).half()
		output = model_trt(image).detach().cpu().numpy().flatten()
		goal_x = float(output[0])  #采集图片给模型，输出当前小车朝向点距离图像中心点的偏移位置

def registerScan(scan_data):
	global minranges
	global min_angleX
	minranges = scan_data.distance
	min_angleX = scan_data.angleX
	if min_angleX>0:
		min_angleX=3.1415-min_angleX;
	else:
		min_angleX=-(min_angleX+3.1415);
	
def run():
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
	global stop_x
	global stop_y
	global autodrive
	global minranges
	global min_angleX
	global normal_vel
	rate = rospy.Rate(20)
	time.sleep(10.0)
	ii = 0
	while not rospy.is_shutdown():
		if minranges<0.18 and abs(min_angleX)<1.8:        #障碍物 <0.18m ,the car stop
			pub_cmd(0.0,0.0)
			time.sleep(0.1)
			continue
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
			autodrive = 1
		rate.sleep()

def main():
	global model_trt
	global normal_vel
	global STEERING_GAIN
	global STEERING_BIAS
	global if_akm_yes_or_no
	rospy.init_node("wheeltec_jetracer")
	print("please wait a minute")
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
	#image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

	normal_vel= rospy.get_param('~normal_vel') 
	STEERING_GAIN= rospy.get_param('~STEERING_GAIN') 
	STEERING_BIAS= rospy.get_param('~STEERING_BIAS') 
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


