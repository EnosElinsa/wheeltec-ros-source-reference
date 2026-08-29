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

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from radar_msgs.msg import RadarScan,RadarReturn
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from rclpy.qos import QoSProfile,QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String as StringMsg	
from math import sqrt,atan2,sin
class LaserTracker(Node):

	def __init__(self):
		super().__init__('radarTracker')
		self.lastScan=None
		# qos = QoSProfile(depth=10)
		self.declare_parameter('is_track_radarid',False)
		self.declare_parameter('disable_distance_x',10.0)
		self.is_track_radarid = False
		self.disable_distance_x = self.get_parameter('disable_distance_x').get_parameter_value().double_value
		self.positionPublisher = self.create_publisher(PositionMsg, 'object_tracker/current_position', 10)
		self.infoPublisher = self.create_publisher(StringMsg, 'object_tracker/info', 10)
		self.scanSubscriber = self.create_subscription(
		    RadarScan,
		    '/radarscan',
		    self.registerScan,
			qos_profile=qos_profile_sensor_data)
		self.track_id = 0
		self.old_distance = 1.5
		
	def registerScan(self, data):
		# 初始化最小距离为无穷大，表示尚未找到任何目标
		min_distance = float('inf')
		track_min_distance = float('inf')
		# 初始化最小距离对应的角度为0.0
		minDistanceAngle = 0.0
		track_minDistanceAngle = 0.0
		minDistancetid = 0
		# 检查data.returns是否存在数据
		if data.returns:	
			# 遍历data.returns中的每一个对象
			for obj in data.returns:
				# 获取当前对象的距离、方位角（角度）
				distance = obj.range
				distanceAngle = obj.azimuth
				yy = distance * sin(distanceAngle)
				if abs(yy) > self.disable_distance_x:
					continue
				if self.track_id == obj.amplitude and abs(self.old_distance - distance) < 2.0:
				#if abs(self.old_distance - distance) < 0.25:
					track_min_distance = distance
					track_minDistanceAngle = distanceAngle
				if distance < min_distance:
					minDistancetid = obj.amplitude
					min_distance = distance
					minDistanceAngle = distanceAngle
			if self.is_track_radarid == True:
				print(self.track_id)
				if track_min_distance != float('inf'):
					min_distance = track_min_distance
					minDistanceAngle = track_minDistanceAngle
				else:                  #cannot find track id
					for obj in data.returns:
						distance = obj.range
						distanceAngle = obj.azimuth
						yy = distance * sin(distanceAngle)
						if abs(self.old_distance - distance) < 2.0 and abs(yy)<self.disable_distance_x:
							min_distance = distance
							minDistanceAngle = distanceAngle
							self.track_id = obj.amplitude
		if min_distance == float('inf'):
			min_distance = 10.0
			minDistanceAngle = 0.0
		# 创建一个PositionMsg类型的消息对象
		msgdata = PositionMsg()
		msgdata.angle_x = minDistanceAngle
		# 打印最小距离及角度
		print(min_distance)
		print(minDistanceAngle/3.14*180)
		# 发布最小距离消息
		self.old_distance = min_distance
		self.old_angle = minDistanceAngle
		msgdata.distance = float(min_distance)
		self.positionPublisher.publish(msgdata)
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    lasertracker = LaserTracker()
    print('seem to do something')
    try:
        rclpy.spin(lasertracker)
    finally:
        lasertracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
