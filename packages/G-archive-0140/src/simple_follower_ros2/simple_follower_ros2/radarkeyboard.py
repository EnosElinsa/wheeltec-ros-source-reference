#!/usr/bin/env python
# coding=utf-8


import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile,QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
from turn_on_wheeltec_robot.msg import Position as PositionMsg

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

#紧急刹停距离：m
stop_distance = 1.5

minobject = 100.0 

msg = """
Control Your carrrrrrrrrr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""
e = """
Communications Failed
"""
#键值对应移动/转向方向
moveBindings = {
        'i':( 1, 0),
        'o':( 1,-1),
        'j':( 0, 1),
        'l':( 0,-1),
        'u':( 1, 1),
        ',':(-1, 0),
        '.':(-1,1),
        'm':(-1,-1),
           }

#键值对应速度增量
speedBindings={
        'q':(1.1,1.1),
        'z':(0.9,0.9),
        'w':(1.1,1),
        'x':(0.9,1),
        'e':(1,  1.1),
        'c':(1,  0.9),
          }
#获取键值函数
speed = 0.2 #默认移动速度 m/s
turn  = 1   #默认转向速度 rad/

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#以字符串格式返回当前速度
def print_vels(speed, turn):
    print('currently:\tspeed {0}\t turn {1} '.format(
        speed,
        turn))
def registerScan(objects):
    global minobject 
    if objects:
        minobject = objects.distance
    else:
        minobject = 100


def main():
    global minobject 
    global stop_distance
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('radarkeyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    scanSubscriber = node.create_subscription(
		    PositionMsg,
		    'object_tracker/current_position',
		    registerScan,
			qos_profile=qos_profile_sensor_data)
    speed = 0.2 #默认移动速度 m/s
    turn  = 1.0   #默认转向速度 rad/
    x      = 0.0   #前进后退方向
    th     = 0.0   #转向/横向移动方向
    count  = 0.0   #键值不再范围计数
    target_speed = 0.0 #前进后退目标速度
    target_turn  = 0.0 #转向目标速度
    target_HorizonMove = 0.0 #横向移动目标速度
    control_speed = 0.0 #前进后退实际控制速度
    control_turn  = 0.0 #转向实际控制速度
    control_HorizonMove = 0.0 #横向移动实际控制速度
    Omni = 0
    try:
        print(msg)
        print(print_vels(speed, turn))
        while(1):
            key = get_key(settings)
            #切换是否为全向移动模式，全向轮/麦轮小车可以加入全向移动模式
            if key=='b':               
                Omni=~Omni
                if Omni: 
                    print("Switch to OmniMode")
                    moveBindings['.']=[-1,-1]
                    moveBindings['m']=[-1, 1]
                else:
                    print("Switch to CommonMode")
                    moveBindings['.']=[-1, 1]
                    moveBindings['m']=[-1,-1]
            
            #判断键值是否在移动/转向方向键值内
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            #判断键值是否在速度增量键值内
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(print_vels(speed,turn)) #速度发生变化，打印出来

            #空键值/'k',相关变量置0
            elif key == ' ' or key == 'k' :
                x  = 0
                th = 0.0
                control_speed = 0.0
                control_turn  = 0.0
                HorizonMove   = 0.0

            #长期识别到不明键值，相关变量置0
            else:
                count = count + 1
                if count > 4:
                    x  = 0
                    th = 0.0
                if (key == '\x03'):
                    break

           #根据速度与方向计算目标速度
            target_speed = speed * x
            target_turn  = turn * th
            target_HorizonMove = speed*th

            #平滑控制，计算前进后退实际控制速度
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed

            #平滑控制，计算转向实际控制速度
            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            #平滑控制，计算横向移动实际控制速度
            if target_HorizonMove > control_HorizonMove:
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                control_HorizonMove = target_HorizonMove
         
            twist = Twist() #创建ROS速度话题变量
            twist.linear.x  = control_speed; twist.linear.y = 0.0;  twist.linear.z = 0.0
            twist.angular.x = 0.0;             twist.angular.y = 0.0; twist.angular.z = control_turn

            #创建小车最终要发布小车速度的消息类型，初始线速度角速度为0（小车停止状态）
            twist_pub = Twist()
            #如果最近障碍物距离大于刹停距离的，或者是倒车的，则赋值并发布小车速度，小车运动，不满足条件的，小车赋值发布的小车速度为0，小车停止运动             
            if minobject < stop_distance and twist.linear.x > 0.0:
                twist.linear.x = 0.0
            pub.publish(twist)
            rclpy.spin_once(node,timeout_sec=0.02)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
