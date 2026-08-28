#!/usr/bin/env python 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作
#2:Python.源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

#引用ros库
import rospy

# import xxx as xxx :给模块做别名
# from xxx import xxx:从模块中导入某个变量

# 用到的变量定义
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from turtlesim.srv import Spawn

# 用于记录充电桩位置、发布导航点
from geometry_msgs.msg import PoseStamped

# 中断导航相关
from actionlib_msgs.msg import GoalID

# rviz可视化相关
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# cmd_vel话题数据
from geometry_msgs.msg import Twist

# 里程计话题相关
from nav_msgs.msg import Odometry

# 获取导航结果
from move_base_msgs.msg import MoveBaseActionResult

# tf坐标相关，未使用
import tf, tf2_ros

# 键盘控制相关
import sys, select, termios, tty

# 延迟相关
import time

# 读写充电桩位置文件
import json

import math

#存放充电桩位置的文件位置
json_file='/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/Charger_Position.json'

#print_and_fixRetract相关，用于打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

#圆周率
PI=3.1415926535897

#获取键值初始化，读取终端相关属性
settings = termios.tcgetattr(sys.stdin)
def getKey():
	'''获取键值函数'''
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def print_and_fixRetract(str):
	'''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print(str)

class AutoRecharger():
	def __init__(self):
		#创建节点
		rospy.init_node("auto_recharger") 
		print_and_fixRetract('Automatic charging node start!')

		#机器人状态变量：类型、电池容量、电池电压、充电状态、充电电流、红外信号状态、记录机器人姿态
		self.robot = {
			'Type':'Plus', 
			'BatteryCapacity':5000, 
			'Voltage':25, 
			'Charging':0, 
			'Charging_current':0, 
			'RED':0, 
			'Rotation_Z':0,
			'car_mode':'mini_mec'
		}

		#用于记录导航结束是的机器人Z轴姿态
		self.nav_end_z=0
		self.start_turn = 0
		self.find_redsignal = 0

		#红外信号的数量
		self.red_count=0
		#机器人自动回充模式标志位，0：关闭回充，1：导航回充，2：回充装备控制回充
		self.chargeflag=0
		#机器人时间戳记录变量
		self.last_time=rospy.Time.now()
		#机器人红外信号丢失的时间滤波2
		self.lost_red_flag=rospy.Time.now()
		#机器人电量过低(<12.5或者<25)计数
		self.power_lost_count=0
		#机器人低电量检测1次标志位
		self.lost_power_once=1
		#机器人充电完成标志位
		self.charge_complete=0
		#机器人充电完成标志位
		self.last_charge_complete=0
		#最新充电桩位置数据
		self.json_data=0
		#是否接受导航结束回调
		self.nav_end_rc_flag=0

		# 用户标记点和实际导航点直线偏移的距离，单位m
		self.diff_point = 1.2

		# 偏移的角度
		self.diff_angle = -15

		#读取json文件内保存的充电桩位置信息
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)

		#创建充电桩位置目标点话题发布者
		self.Charger_Position_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
		#创建导航中断话题发布者
		self.NavGoal_Cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=5)
		#创建充电桩位置标记话题发布者
		self.Charger_marker_pub   = rospy.Publisher('path_point', MarkerArray, queue_size = 100) 
		#创建自动回充任务是否开启标志位话题发布者
		self.Recharger_Flag_pub = rospy.Publisher("robot_recharge_flag", Int8, queue_size=5)
		#速度话题用于不开启导航时，向底盘发送开启自动回充任务命令
		self.Cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
		#创建设置红外对接速度话题的发布者
		# self.Red_Vel_pub = rospy.Publisher("red_vel", Twist, queue_size=5)
		#创建多点导航终止话题的发布者
		self.Over_point_nav = rospy.Publisher("nav_over", Bool, queue_size=1)
		#创建机器人电量话题订阅者
		self.Voltage_sub = rospy.Subscriber("PowerVoltage", Float32, self.Voltage_callback)
		#创建机器人充电状态话题订阅者
		self.Charging_Flag_sub = rospy.Subscriber("robot_charging_flag", Bool, self.Charging_Flag_callback)
		#创建机器人充电电流话题订阅者
		self.Charging_Current_sub = rospy.Subscriber("robot_charging_current", Float32, self.Charging_Current_callback)
		#创建机器人已发现红外信号话题订阅者
		self.RED_Flag_sub = rospy.Subscriber("robot_red_flag", UInt8, self.RED_Flag_callback)
		#创建充电桩位置更新话题订阅者
		self.Charger_Position_Update_sub = rospy.Subscriber("charger_position_update", PoseStamped, self.Position_Update_callback)
		#创建导航结果话题订阅者
		self.Goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.Nav_Result_callback)
		#创建里程计话题订阅者
		self.Odom_sub = rospy.Subscriber('/odom', Odometry, self.Odom_callback)

		# 创建服务调用者
		self.set_charge = rospy.ServiceProxy('/set_charge', Spawn)

		#按键控制说明
		self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start Navigation to find charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
		"""

	# 检查是否存在对应的话题
	def check_topic(self,topic_name):
		topics = rospy.get_published_topics()
		for i in topics:
			if i[0] == topic_name:
				return True
		return False

	# 清除多点导航
	def clear_allNav(self):
		topic = Bool()
		topic.data=True
		for i in range(10):
			self.Over_point_nav.publish(topic)

	# 设置自动回充的状态
	def set_charge_mode(self,value,max_callcount=10):
	
		try:
			rospy.wait_for_service('/set_charge',timeout=2)  # 等待服务可用
		except rospy.ROSException as e:
			print_and_fixRetract(RED+'开启自动回充功能失败.原因:等待服务超时,请确认底盘节点是否被开启.'+RESET)
			return

		state = None  # 调用结果
		call_time = 0 # 调用失败后尝试调用的次数记录
		while True:
			try:
				response = self.set_charge(x=value)
				state = response.name
			except rospy.ServiceException as e:
				state =  "false"

			if state=="true":
				# 调用成功,跳出循环
				break
			else:
				# 记录失败的次数
				call_time = call_time + 1
				if max_callcount>10:
					print_and_fixRetract(RED+'尝试与底盘通信多次失败,无法开启自动回充功能,请检查底层设备是否正确.'+RESET)
					break
			time.sleep(0.1)
				

	def Pub_Charger_Position(self):
		# 导航到充电桩位置前，发布清除多点导航的话题
		self.clear_allNav()

		# 标记当前是寻找充电桩的导航
		self.nav_end_rc_flag=1

		'''使用最新充电桩位置发布导航目标点话题'''
		nav_goal=PoseStamped()
		nav_goal.header.frame_id = 'map'
		nav_goal.header.stamp = rospy.Time.now()
		nav_goal.pose.position.x = self.json_data['p_x']
		nav_goal.pose.position.y = self.json_data['p_y']
		nav_goal.pose.orientation.z = self.json_data['orien_z']
		nav_goal.pose.orientation.w = self.json_data['orien_w']

		self.Charger_Position_pub.publish(nav_goal)
		self.Pub_Charger_marker(
			self.json_data['p_x'], 
			self.json_data['p_y'], 
			self.json_data['orien_z'], 
			self.json_data['orien_w'])

	def Pub_NavGoal_Cancel(self):
		'''取消导航'''
		topic=GoalID()
		self.NavGoal_Cancel_pub.publish(topic)

	def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
		'''发布目标点可视化话题'''

		# 实际充电桩位置已经被偏移过，这里可视化转换成用户在rviz上标定的坐标	
		# 角度偏移
		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		tmp_angle = math.radians(-self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		o_z = math.sin(new_yaw)
		o_w = math.cos(new_yaw)

		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		p_x = p_x - diff_x*self.diff_point
		p_y = p_y - diff_y*self.diff_point


		markerArray = MarkerArray()

		marker_shape  = Marker() #创建marker对象
		marker_shape.id = 0 #必须赋值id
		marker_shape.header.frame_id = 'map' #以哪一个TF坐标为原点
		marker_shape.type = marker_shape.ARROW #TEXT_VIEW_FACING #一直面向屏幕的字符格式
		marker_shape.action = marker_shape.ADD #添加marker
		marker_shape.scale.x = 0.5 #marker大小
		marker_shape.scale.y = 0.05 #marker大小
		marker_shape.scale.z = 0.05 #marker大小，对于字符只有z起作用
		marker_shape.color.a = 1 #字符透明度
		marker_shape.color.r = 1 #字符颜色R(红色)通道
		marker_shape.color.g = 0 #字符颜色G(绿色)通道
		marker_shape.color.b = 0 #字符颜色B(蓝色)通道
		marker_shape.pose.position.x = p_x#字符位置
		marker_shape.pose.position.y = p_y #字符位置
		marker_shape.pose.position.z = 0.1 #msg.position.z #字符位置
		marker_shape.pose.orientation.z = o_z #字符位置
		marker_shape.pose.orientation.w = o_w #字符位置
		markerArray.markers.append(marker_shape) #添加元素进数组

		marker_string = Marker() #创建marker对象
		marker_string.id = 1 #必须赋值id
		marker_string.header.frame_id = 'map' #以哪一个TF坐标为原点
		marker_string.type = marker_string.TEXT_VIEW_FACING #一直面向屏幕的字符格式
		marker_string.action = marker_string.ADD #添加marker
		marker_string.scale.x = 0.5 #marker大小
		marker_string.scale.y = 0.5 #marker大小
		marker_string.scale.z = 0.5 #marker大小，对于字符只有z起作用
		marker_string.color.a = 1 #字符透明度
		marker_string.color.r = 1 #字符颜色R(红色)通道
		marker_string.color.g = 0 #字符颜色G(绿色)通道
		marker_string.color.b = 0 #字符颜色B(蓝色)通道
		marker_string.pose.position.x = p_x #字符位置
		marker_string.pose.position.y = p_y #字符位置
		marker_string.pose.position.z = 0.1 #msg.position.z #字符位置
		marker_string.pose.orientation.z = o_z #字符位置
		marker_string.pose.orientation.w = o_w #字符位置
		marker_string.text = 'Charger' #字符内容
		markerArray.markers.append(marker_string) #添加元素进数组
		self.Charger_marker_pub.publish(markerArray) #发布markerArray，rviz订阅并进行可视化

	def Pub_Recharger_Flag(self,set_velflag=0):
		'''发布自动回充任务是否开启标志位话题'''

		# 设置自动回充状态
		self.set_charge_mode(self.chargeflag)

		# 传递标志位,作用:需要优先开启自动回充然后再进行导航时,需要传递此标志位
		if set_velflag==1:
			topic=Int8()
			topic.data=self.chargeflag
			for i in range(10):
				self.Recharger_Flag_pub.publish(topic)

	def Voltage_callback(self, topic):
		'''更新机器人电池电量'''
		self.robot['Voltage']=topic.data

	def Charging_Flag_callback(self, topic):
		'''更新机器人充电状态'''
		if(self.robot['Charging']==0 and topic.data==1):
			print_and_fixRetract(GREEN+"Charging started!"+RESET)
		if(self.robot['Charging']==1 and topic.data==0):
			print_and_fixRetract(YELLOW+"Charging disconnected!"+RESET)
		self.robot['Charging']=topic.data

	def Charging_Current_callback(self, topic):
		'''更新机器人充电电流数据'''
		self.robot['Charging_current']=topic.data
		
	def RED_Flag_callback(self, topic):
		self.red_count = topic.data
		'''更新是否寻找到红外信号(充电桩)状态'''
		if self.robot['Charging']==0:
			#如果是导航寻找充电桩模式，红外信号消失时
			if topic.data==0 and self.robot['RED']==1:
				if(rospy.Time.now()-self.lost_red_flag).secs>=2:
					print_and_fixRetract(YELLOW+"Infrared signal lost."+RESET)
				self.lost_red_flag = rospy.Time.now()
	
			#红外信号出现
			if topic.data==1 and self.robot['RED']==0:
				print_and_fixRetract(GREEN+"Infrared signal founded."+RESET)

		if topic.data>0:
			self.robot['RED']=1
		else:
			self.robot['RED']=0

		# 自转寻找红外时处理逻辑
		if self.start_turn==1:
			if self.robot['RED']==1:
				self.find_redsignal = self.find_redsignal + 1 
				if self.find_redsignal>=60: # 稳定识别3秒
					self.find_redsignal = 0
					self.start_turn=0
					print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
					vel_topic=Twist()
					self.Cmd_vel_pub.publish(vel_topic) # 停止运动
					self.chargeflag=1 # 开启自动回充
					self.Pub_Recharger_Flag()
					self.nav_end_z = self.robot['Rotation_Z']
			else:
				self.find_redsignal = 0

	def Position_Update_callback(self, topic):
		'''更新json文件中的充电桩位置'''
		position_dic={'p_x':0, 'p_y':0, 'orien_z':0, 'orien_w':0 }
		position_dic['p_x']=topic.pose.position.x
		position_dic['p_y']=topic.pose.position.y
		position_dic['orien_z']=topic.pose.orientation.z
		position_dic['orien_w']=topic.pose.orientation.w

		# 以用户标定的充电桩位置为基础，直线前移1.2米作为真实导航点
		tmp_yaw = math.atan2(2*(position_dic['orien_w']*position_dic['orien_z']),1-2*(position_dic['orien_z']**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		position_dic['p_x'] = position_dic['p_x'] + diff_x*self.diff_point
		position_dic['p_y'] = position_dic['p_y'] + diff_y*self.diff_point

		# 角度偏移
		tmp_angle = math.radians(self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		position_dic['orien_z'] = math.sin(new_yaw)
		position_dic['orien_w'] = math.cos(new_yaw)

		#保存最新的充电桩位置到json文件
		with open(json_file, 'w') as fp:
			json.dump(position_dic, fp, ensure_ascii=False)
			print_and_fixRetract("New charging pile position saved.")
		#更新最新的充电桩位置数据
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)
		#发布最新的充电桩位置话题
		self.Pub_Charger_marker(position_dic['p_x'], position_dic['p_y'], position_dic['orien_z'], position_dic['orien_w'])

	# 导航结果订阅函数
	def Nav_Result_callback(self, topic):
		
		# 目标点取消，一般是多点导航
		if 'canceled' in topic.status.text:
			return

		# 导航点是充电桩位置
		if self.nav_end_rc_flag == 1:
			self.nav_end_rc_flag = 0
			if 'Failed' in topic.status.text:
				print_and_fixRetract('无法导航到充电桩位置,请检查充电桩附近是否存在障碍物.(Cannot navigate to the charging station, please check if there are any obstacles near the charging station.)')
				return
			
			if 'Aborting' in topic.status.text:
				print_and_fixRetract('充电桩位置数据异常,请尝试重新标定.(Charging post position data is abnormal, please try to re-calibrate.)')
				return
			
			if 'oscillating' in  topic.status.text:
				print_and_fixRetract(YELLOW+'WARNING:请清除充电桩附近障碍物.(Please remove obstacles near the charging post.)'+RESET)
				return

			# 导航到达充电桩处或者无法完成充电桩处的导航但是存在红外信号，则开启自动回充
			if self.robot['RED']==1:
				print_and_fixRetract(GREEN+'已到达充电桩位置,开始对接充电.(Arrived at the charging station and started charging.)'+RESET)
				# 开启自动回充
				self.chargeflag=1
				self.Pub_Recharger_Flag()

			else:
				self.lost_power_once=1 # 未找到充电桩时恢复允许低电量导航
				if 'akm' in self.robot['car_mode']:
					print_and_fixRetract(YELLOW+'未找到充电桩,自动回充功能已停止.(Charging station not found.)'+RESET)		
				else:
					# 开启自动回充功能并开始旋转一圈来寻找充电桩
					print_and_fixRetract(YELLOW+'未找到充电桩,开始自转寻找红外信号.(Charging station not found. Starting to rotate in search of infrared signal.)'+RESET)
					self.nav_end_z = self.robot['Rotation_Z']
					time.sleep(1)
					topic=Twist()
					topic.angular.z = 0.2
					self.Cmd_vel_pub.publish(topic) 
					self.start_turn = 1



	def Odom_callback(self, topic):
		'''更新的机器人实时位姿'''
		self.robot['Rotation_Z']=topic.pose.pose.position.z	 

	def Stop_Charge(self):
		#如果在导航回充模式下，关闭导航
		self.Pub_NavGoal_Cancel() 
		self.nav_end_rc_flag=0
		self.lost_power_once=1
		
		#切换为停止回充模式
		self.chargeflag=0
		self.Pub_Recharger_Flag()
		#发布速度为0的话题停止机器人运动
		topic=Twist()
		self.Cmd_vel_pub.publish(topic)
		#如果机器人在充电，控制机器人离开充电桩
		if self.robot['Charging']==1:
			topic=Twist()
			topic.linear.x = 0.1
			self.Cmd_vel_pub.publish(topic)

			# 不同小车电机速度响应时间不同
			if 'mini' in self.robot['car_mode'] or 'akm' in self.robot['car_mode']: 
				time.sleep(3)
			else:
				time.sleep(1)
			topic.linear.x = 0.0
			self.Cmd_vel_pub.publish(topic)

	def autoRecharger(self, key):
		'''键盘控制开始自动回充:1-导航控制寻找充电桩,2-纯回充装备控制寻找充电桩
		'''

		# 如果机器人在充电中,则检测充电是否已经完成.
		if self.robot['Charging']==1:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']>25) or (self.robot['Type']=='Mini' and self.robot['Voltage']>12.5):
				self.charge_complete=self.charge_complete+1
			else:
				self.charge_complete=0

		#导航控制寻找充电桩
		if key=='q' or key=='Q':
			# 存在3路以上的红外信号,小车姿态接近于对准充电桩,无需导航	
			if self.red_count>=3:
				self.Pub_NavGoal_Cancel()
				self.clear_allNav()
				self.chargeflag=1
				self.Pub_Recharger_Flag()
				print_and_fixRetract('已捕获到高强度红外信号,使用红外信号对接.(High-intensity infrared signals have been captured and are docked using infrared signals.)')
			else:
				# 如果存在此话题则判断为用户已经开启导航
				if self.check_topic('/move_base/goal')==True:
					self.Pub_NavGoal_Cancel()
					self.Pub_Charger_Position() 
					print_and_fixRetract('开始导航到充电桩位置.(Start navigating to the charging post location.)')
				else:
					if self.robot['RED']==1:
						self.chargeflag=1
						self.Pub_Recharger_Flag()
						print_and_fixRetract('未检测到导航节点,使用红外信号对接.(No navigation nodes detected, use infrared signals for docking.)')
					else:
						print_and_fixRetract('未检测到导航节点和红外信号,无法启动自动回充模式.(Failure to detect the navigation node and infrared signal, unable to start the automatic recharge mode.)')

		#关闭自动回充
		elif key=='e' or key=='E':
			self.Stop_Charge()				
			print_and_fixRetract('停止寻找充电桩或停止充电.(Stop finding charging pile or charging.)')

		elif key=='p':
			self.set_charge_mode()

		#电压过低时开启导航自动回充
		if self.robot['Charging']==0:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']<20) or (self.robot['Type']=='Mini' and self.robot['Voltage']<10):
				time.sleep(1)
				self.power_lost_count=self.power_lost_count+1 # 低电量滤波

				# 低电量状态超过5次
				if self.power_lost_count>5 and self.lost_power_once==1:
					self.power_lost_count=0

					# 电量低且小车不在回充模式,开启导航充电
					if self.chargeflag==0:
						self.Pub_NavGoal_Cancel() # 取消导航
						# 检测是否存在导航节点
						if self.check_topic('/move_base/goal')==True:
							if 'akm' in self.robot['car_mode']:
								self.chargeflag=2 # 阿克曼车型与其他车型低电量导航底层逻辑不一样
							else:
								self.chargeflag=1
							self.Pub_Recharger_Flag(1) # 出现要优先开启自动回充然后再导航的情况,需要进行标志位传递
							self.Pub_Charger_Position()
							print_and_fixRetract(YELLOW+'检测到电池电量低,即将导航到充电桩进行充电.(Detects low battery level and will navigate to a charging station for charging.)'+RESET)
							self.lost_power_once=0
						else:
							if self.robot['RED']==1:
								self.chargeflag=1
								self.Pub_Recharger_Flag()
								print_and_fixRetract(YELLOW+'电池电量低,未检测到导航节点,使用红外信号对接.(Low battery, no navigation node detected, use infrared signal for docking.)'+RESET)
								self.lost_power_once=0	
							else:
								print_and_fixRetract(RED+'电池电量低,未检测到导航节点和红外信号,无法自动开启自动回充功能.(Battery power is low, navigation nodes and infrared signals are not detected, and the auto-recharge function cannot be turned on automatically.)'+RESET)		
			else:
				self.power_lost_count=0		

		#频率1hz的循环任务
		if (rospy.Time.now()-self.last_time).secs>=1:
			#发布充电桩位置话题
			self.Pub_Charger_marker(
				self.json_data['p_x'], 
				self.json_data['p_y'], 
				self.json_data['orien_z'], 
				self.json_data['orien_w'])
			self.last_time=rospy.Time.now()

			#充电期间打印电池电压、充电时间
			if self.robot['Charging']==1:
				self.lost_power_once=1
				percent=0
				percen_form=0
				if self.robot['Type']=='Plus':
					percent= (self.robot['Voltage']-20)/5
					percent_form=format(percent, '.0%')
				if self.robot['Type']=='Mini':
					percent= (self.robot['Voltage']-10)/2.5
					percent_form=format(percent, '.0%')
				print_and_fixRetract("Robot is charging.")
				print_and_fixRetract("Robot battery: "+str(round(self.robot['Voltage'], 2))+"V = "+str(percent_form)+
									 ", Charging current: "+str(round(self.robot['Charging_current'], 2))+"A.")
				mAh_time=0
				try:
					mAh_time=1/self.robot['Charging_current']/1000
				except ZeroDivisionError:
					pass
				left_battery=round(self.robot['BatteryCapacity']*percent, 2)
				if percent<1:
					need_charge_battery=self.robot['BatteryCapacity']-left_battery
					need_percent_form=format(1-percent, '.0%')		
					print_and_fixRetract(str(self.robot['BatteryCapacity'])+"mAh*"+str(need_percent_form)+"="+str(need_charge_battery)+"mAh need to be charge, "+
										 "cost "+str(round(need_charge_battery*mAh_time, 2))+" hours.")
				else:	
					print_and_fixRetract(GREEN+"Robot battery is full."+RESET)
				print_and_fixRetract("\n")

			# 非阿克曼车型如果导航到终点没有红外信号,则自传一圈寻找
			if self.start_turn == 1:
				if abs(self.robot['Rotation_Z']-self.nav_end_z)>2*PI:
					self.start_turn=0
					self.Stop_Charge()
					print_and_fixRetract(RED+'自转已完成,无法找到充电桩位置,已停止自动回充.(Rotation completed, unable to locate charging station, automatic recharging has been stopped.)'+RESET)

		#机器人充电完成判断
		if self.charge_complete>10:
			self.charge_complete=0
			if self.last_charge_complete!=0:
				self.last_charge_complete=0
				self.Stop_Charge()			
			print_and_fixRetract(GREEN+'充电已完成.(Chrge complete.)'+RESET)#Charging complete
		self.last_charge_complete=self.charge_complete

if __name__ == '__main__':
	try:
		AutoRecharger=AutoRecharger() #创建自动回充类
		print_and_fixRetract(AutoRecharger.tips)

		#从参数服务器获取参数
		AutoRecharger.robot['BatteryCapacity'] = rospy.get_param("/robot_BatteryCapacity", default="5000")
		AutoRecharger.robot['car_mode']        = rospy.get_param("/wheeltec_robot/car_mode",default='mini_mec')
		AutoRecharger.diff_point = rospy.get_param("~diff_point",default='1.2')
		AutoRecharger.diff_angle = rospy.get_param("~diff_angle",default='-15')

		if AutoRecharger.robot['car_mode'][0:4]!='mini':
			AutoRecharger.robot['Type'] = 'Plus'
		else:
			AutoRecharger.robot['Type'] = 'Mini'

		while  not rospy.is_shutdown():
			key = getKey() #获取键值，会导致终端打印自动缩进
			AutoRecharger.autoRecharger(key) #开始自动回充功能
			if (key == '\x03'):
				topic=Twist()
				AutoRecharger.Cmd_vel_pub.publish(topic) #发布速度0话题
				AutoRecharger.chargeflag=0
				AutoRecharger.Pub_Recharger_Flag(1)# 关闭自动回充
				print_and_fixRetract('自动回充功能已关闭.(Quit AutoRecharger.)')#Auto charging quit
				break #Ctrl+C退出自动回充功能
		
	except rospy.ROSInterruptException:
		print_and_fixRetract('exception')
	
	finally:
		topic=Twist()
		AutoRecharger.Cmd_vel_pub.publish(topic) #发布速度0话题
		AutoRecharger.chargeflag=0
		AutoRecharger.Pub_Recharger_Flag(1)# 关闭自动回充,并传递标志位到cmd_vel的callback函数
		# 恢复终端属性
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
