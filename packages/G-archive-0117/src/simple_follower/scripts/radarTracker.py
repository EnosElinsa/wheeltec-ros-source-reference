#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import time
import numpy as np
from wheeltec_radar.msg import RadarDetectionArray,RadarDetection
from simple_follower.msg import position as PositionMsg
from std_msgs.msg import String as StringMsg	
from math import sqrt,atan2
class LaserTracker():

	def __init__(self):
		self.is_track_radarid = rospy.get_param('~is_track_radarid',False)
		self.disable_distance_x = rospy.get_param('~disable_distance_x',10.0)
		self.scanSubscriber = rospy.Subscriber('radarscan', RadarDetectionArray, self.registerScan)
		self.positionPublisher = rospy.Publisher('object_tracker/current_position', PositionMsg,queue_size=3)
		self.infoPublisher = rospy.Publisher('object_tracker/info', StringMsg, queue_size=3)
		self.track_id = 0
		self.old_distance = 1.0
		self.old_angle = 0.0
		


	def registerScan(self, data):
		# 初始化最小距离为无穷大，表示尚未找到任何目标
		min_distance = float('inf')
		track_min_distance = float('inf')
		# 初始化最小距离对应的角度为0.0
		minDistanceAngle = 0.0
		track_minDistanceAngle = 0.0
		minDistancetid = 0
		# 检查data.returns是否存在数据
		if data.detections:	
			# 遍历data.returns中的每一个对象
			for obj in data.detections:
				# 获取当前对象的距离、方位角（角度）
				xx = obj.position.x
				yy = obj.position.y
				distance = sqrt(xx * xx + yy * yy)
				distanceAngle = atan2(yy, xx)
				if abs(yy) > self.disable_distance_x:
					continue
				if self.track_id == obj.detection_id and abs(self.old_distance - distance) < 2.0:
				#if abs(self.old_distance - distance) < 0.25:
					track_min_distance = distance
					track_minDistanceAngle = distanceAngle
				if distance < min_distance:
					minDistancetid = obj.detection_id
					min_distance = distance
					minDistanceAngle = distanceAngle
			if self.is_track_radarid == True:
				print(self.track_id)
				if track_min_distance != float('inf'):
					min_distance = track_min_distance
					minDistanceAngle = track_minDistanceAngle
				else:                  #cannot find track id
					for obj in data.detections:
						xx = obj.position.x
						yy = obj.position.y
						distance = sqrt(xx * xx + yy * yy)
						distanceAngle = atan2(yy, xx)
						if abs(self.old_distance - distance) < 2.0 and abs(yy)<self.disable_distance_x:
							min_distance = distance
							minDistanceAngle = distanceAngle
							self.track_id = obj.detection_id
		if min_distance == float('inf'):
			min_distance = 10.0
			minDistanceAngle = 0.0
		# 创建一个PositionMsg类型的消息对象
		msgdata = PositionMsg()
		msgdata.angleX = minDistanceAngle
		# 打印最小距离及角度
		#print(min_distance)
		#print(minDistanceAngle/3.14*180)
		# 发布最小距离消息
		self.old_distance = min_distance
		self.old_angle = minDistanceAngle
		msgdata.distance = float(min_distance)
		self.positionPublisher.publish(msgdata)
def main(args=None):
    print('starting')
    rospy.init_node('radarTracker')
    lasertracker = LaserTracker()
    print('seem to do something')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')


if __name__ == '__main__':
    main()
