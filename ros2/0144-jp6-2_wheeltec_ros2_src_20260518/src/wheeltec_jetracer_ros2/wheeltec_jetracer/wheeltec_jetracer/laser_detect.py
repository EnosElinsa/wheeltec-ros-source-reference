#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from wheeltec_jetracer_msg.msg import LaserPosition

class LaserDetect(Node):
    def __init__(self):
        super().__init__('laser_detect')
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.register_scan,
            10
        )
        self.position_publisher = self.create_publisher(
            LaserPosition,
            '/laser_distance',
            3
        )
        
        # 添加调试日志级别
        self.get_logger().info('Laser detection node started')
        # 首次收到数据前记录等待状态
        self.first_scan_received = False

    def get_range_at_angle(self, ranges, angle_min, angle_increment, target_angle):
        """
        安全地获取指定角度的距离值
        target_angle: 弧度，以雷达坐标系为准（通常0为正前方，逆时针增加）
        """
        # 计算索引
        idx = int((target_angle - angle_min) / angle_increment)
        
        # 边界检查
        if idx < 0 or idx >= len(ranges):
            self.get_logger().warning(
                f'Calculated index {idx} out of bounds [0, {len(ranges)}). '
                f'Angle {target_angle:.2f} rad, min {angle_min:.2f}, max {angle_min + len(ranges)*angle_increment:.2f}'
            )
            return -1.0
        
        # 获取周围数据的平均值（平滑噪声）
        window_size = 5
        start_idx = max(0, idx - window_size//2)
        end_idx = min(len(ranges), idx + window_size//2 + 1)
        
        valid_ranges = []
        for i in range(start_idx, end_idx):
            r = ranges[i]
            # 严格过滤无效值：大于0，不是nan，不是inf，不太远
            if 0.0 < r < 10.0 and not math.isnan(r) and not math.isinf(r):
                valid_ranges.append(r)
        
        if valid_ranges:
            return float(np.mean(valid_ranges))
        else:
            return -1.0

    def register_scan(self, scan_data):
        ranges = np.array(scan_data.ranges)
        
        if len(ranges) == 0:
            self.get_logger().warning('Received empty laser scan')
            return
            
        if not self.first_scan_received:
            self.get_logger().info(
                f'First scan received: {len(ranges)} points, '
                f'angle_min: {scan_data.angle_min:.2f}, '
                f'angle_max: {scan_data.angle_min + len(ranges)*scan_data.angle_increment:.2f}, '
                f'increment: {scan_data.angle_increment:.4f}'
            )
            self.first_scan_received = True
        
        # 常用激光雷达坐标系：
        # 0度通常为正前方（X轴正方向）
        angle_front = 0.0
        angle_right_5 = math.radians(-5)
        angle_left_45 = math.radians(45)
        angle_left_75 = math.radians(75)
        
        # 尝试两种常见约定：
        # 先尝试0度在前方（标准右手坐标系）
        dist_front = self.get_range_at_angle(
            ranges, scan_data.angle_min, scan_data.angle_increment, angle_front
        )
        
        # 如果前方是-1，可能是雷达范围不同，尝试其他方法
        if dist_front < 0:
            # 尝试将0映射到数组中间（如果是-π到+π范围）
            mid_idx = len(ranges) // 2
            dist_front = float(ranges[mid_idx]) if 0 < ranges[mid_idx] < 10 else -1.0
        
        # 右侧5度
        dist_r5 = self.get_range_at_angle(
            ranges, scan_data.angle_min, scan_data.angle_increment, angle_right_5
        )
        
        dist_l45 = self.get_range_at_angle(
            ranges, scan_data.angle_min, scan_data.angle_increment, angle_left_45
        )
        
        dist_l75 = self.get_range_at_angle(
            ranges, scan_data.angle_min, scan_data.angle_increment, angle_left_75
        )
        # 发布数据
        position_msg = LaserPosition()
        position_msg.distance_front = dist_front
        position_msg.distance_r5 = dist_r5
        position_msg.distance_l45 = dist_l45
        position_msg.distance_l75 = dist_l75
        self.position_publisher.publish(position_msg)
        
        # 调试输出（每10次打印一次避免日志刷屏）
        if hasattr(self, 'counter'):
            self.counter += 1
        else:
            self.counter = 0
            
        #if self.counter % 10 == 0:
        #    self.get_logger().info(
        #        f'Front: {dist_front:.2f}m, Right5: {dist_r5:.2f}m, Left45: {dist_l45:.2f}m, Left75: {dist_l75:.2f}m'
        #    )

def main(args=None):
    rclpy.init(args=args)
    node = LaserDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
