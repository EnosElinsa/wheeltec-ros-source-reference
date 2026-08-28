#!/usr/bin/env python3
# coding: utf-8


import rospy
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from wheeltec_jetracer.msg import laser_position
		
class laserDetect:
	def __init__(self):
		self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.positionPublisher = rospy.Publisher('/laser_distance', laser_position,queue_size=3)

	def registerScan(self, scan_data):
		ranges = np.array(scan_data.ranges)
		temp = []
		ID_fornt = None
		ID_r5 = None
		# ID_45 = None
		# ID_60 = None
		# ID_90 = None

		Distance_fornt   = float('inf')	
		Distance_r5   = float('inf')	
		# Distance_45   = float('inf')
		# Distance_60   = float('inf')	
		# Distance_90   = float('inf')		


		if(not(ranges is None)):
			ID_fornt = int((3.14 - scan_data.angle_min) / scan_data.angle_increment)-1
			for i in ranges[ID_fornt-9:ID_fornt+1]:
				if i < 10:
					temp.append(i)
			if len(temp)!= 0:
				Distance_fornt = sum(temp)/len(temp)
			else:
				Distance_fornt = -1
			temp = []

#------------------------------------------------------
# 左边负数；右边正数
			ID_r5 = int(( 3.053 - scan_data.angle_min) / scan_data.angle_increment) -1
			for i in ranges[ID_r5-5 : ID_r5+5]:
				if i < 10:
					temp.append(i)
			if len(temp)!= 0:
				Distance_r5 = sum(temp)/len(temp)
			else:
				Distance_r5 = -1
			temp = []

#------------------------------------------------------

# 			ID_45 = int((-2.35 - scan_data.angle_min) / scan_data.angle_increment) -1
# 			for i in ranges[ID_45-2 : ID_45+3]:
# 				if i < 5:
# 					temp.append(i)
# 			if len(temp)!= 0:
# 				Distance_45 = sum(temp)/len(temp)
# 			else:
# 				Distance_45 = -1
# 			temp = []
# #------------------------------------------------------
# 			ID_60 = int((-1.74 - scan_data.angle_min) / scan_data.angle_increment) -1
# 			for i in ranges[ID_60-2 : ID_60+3]:
# 				if i < 5:
# 					temp.append(i)
# 			if len(temp)!= 0:
# 				Distance_60 = sum(temp)/len(temp)
# 			else:
# 				Distance_60 = -1
# 			temp = []
# #------------------------------------------------------
# 			ID_90 = int((-1.58 - scan_data.angle_min) / scan_data.angle_increment) -1
# 			for i in ranges[ID_90-2 : ID_90+3]:
# 				if i < 5:
# 					temp.append(i)
# 			if len(temp)!= 0:
# 				Distance_90 = sum(temp)/len(temp)
# 			else:
# 				Distance_90 = -1
# 			temp = []
#------------------------------------------------------
		pass
		self.positionPublisher.publish(laser_position(Distance_fornt, Distance_r5))


if __name__ == '__main__':
	print('starting')
	rospy.init_node('laser_detect')
	detect = laserDetect()
	#print('seems to do something')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


