#!/usr/bin/env python
# coding=utf-8

import rospy

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

def security_publisher():

	rospy.init_node("security_node") 

	security_pub = rospy.Publisher("chassis_security", Int8, queue_size=5)

	cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)

	rate = rospy.Rate(1)

	times = 0

	while not rospy.is_shutdown():

		topic = Int8()
		topic.data = 1
		security_pub.publish(topic)

		if times<3:
			times = times + 1
			if times == 2:
				twist = Twist()
				cmd_vel_pub.publish(twist)
				print(YELLOW+"The active safety protection of the robot has been cancelled"+RESET)

		rate.sleep()


if __name__ == '__main__':
	
	security_publisher()
