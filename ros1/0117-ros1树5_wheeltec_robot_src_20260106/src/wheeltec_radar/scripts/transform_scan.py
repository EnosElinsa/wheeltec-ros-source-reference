#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from wheeltec_radar.msg import RadarDetection, RadarDetectionArray
import math

class LaserScanModifier:
    def __init__(self):
        # 初始化节点
        rospy.init_node('transform_scan', anonymous=True)

        # 订阅 RadarScan 话题
        self.subscription = rospy.Subscriber('/radarscan', RadarDetectionArray, self.listener_callback, queue_size=10)

        # 发布修改后的 LaserScan
        self.publisher = rospy.Publisher('/scan2', LaserScan, queue_size=10)

        # 初始化 LaserScan 消息模板
        self.modified_scan = LaserScan()
        self.modified_scan.angle_min = 0.0
        self.modified_scan.angle_max = 2 * math.pi
        self.modified_scan.angle_increment = 2 * math.pi / 720.0
        self.modified_scan.range_min = 0.0
        self.modified_scan.range_max = 30.0
        self.modified_scan.ranges = [float('inf')] * 720

    def listener_callback(self, msg):
        # 重置 ranges 为无穷远
        self.modified_scan.ranges = [float('inf')] * 720
        self.modified_scan.header = msg.header  # 复用雷达的时间戳和 frame_id

        # 遍历每个雷达目标
        for obj in msg.detections:
            # 计算角度索引（注意：ROS 中角度通常为 [-π, π] 或 [0, 2π]，需确保一致）
            xx = obj.position.x
            yy = obj.position.y
            dis =  math.sqrt(xx * xx + yy * yy)
            angle = math.atan2(yy, xx)
            angle_diff = angle - self.modified_scan.angle_min
            index = int(round(angle_diff / self.modified_scan.angle_increment))

            for i in range(-3,3):
                if self.modified_scan.ranges[index+i] > dis:
                    self.modified_scan.ranges[index+i] = dis

        # 发布
        self.publisher.publish(self.modified_scan)

def main():
    modifier = LaserScanModifier()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

