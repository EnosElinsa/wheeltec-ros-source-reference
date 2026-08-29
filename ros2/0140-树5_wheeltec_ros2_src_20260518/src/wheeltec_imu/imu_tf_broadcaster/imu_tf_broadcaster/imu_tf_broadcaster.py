#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_data_to_tf')
        
        # Declare parameters with default values
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('position_x', 1)
        self.declare_parameter('position_y', 1)
        self.declare_parameter('position_z', 0)
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('imu_frame_id', 'imu')
        
        # Get parameter values
        self.imu_topic = self.get_parameter('imu_topic').value
        self.position_x = self.get_parameter('position_x').value
        self.position_y = self.get_parameter('position_y').value
        self.position_z = self.get_parameter('position_z').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create IMU subscriber
        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10)
        
    def imu_callback(self, msg):
        t = TransformStamped()
        
        # Fill in the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.imu_frame_id
        
        # Set the translation
        t.transform.translation.x = float(self.position_x)
        t.transform.translation.y = float(self.position_y)
        t.transform.translation.z = float(self.position_z)
        
        # Set the rotation from the IMU message
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()