#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import geometry_msgs
import tf

# 初始化变量
position_x = 0
position_y = 0
position_z = 0
imu_frame_id = ""
world_frame_id = ""


def imu_callback(imu_data):
    # 创建广播器
    br = tf2_ros.TransformBroadcaster()

    # 创建变换
    t = geometry_msgs.msg.TransformStamped()

    # 设置时间戳、父坐标系和子坐标系
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = world_frame_id
    t.child_frame_id = imu_frame_id

    # 设置平移部分
    t.transform.translation.x = position_x
    t.transform.translation.y = position_y
    t.transform.translation.z = position_z

    # 从 IMU 消息包中获取四元数数据并归一化
    q = (
        imu_data.orientation.x,
        imu_data.orientation.y,
        imu_data.orientation.z,
        imu_data.orientation.w
    )
    q_normalized = tf.transformations.unit_vector(q)

    # 设置旋转部分
    t.transform.rotation.x = q_normalized[0]
    t.transform.rotation.y = q_normalized[1]
    t.transform.rotation.z = q_normalized[2]
    t.transform.rotation.w = q_normalized[3]

    # 广播变换
    br.sendTransform(t)


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('imu_data_to_tf')

    # 从参数服务器获取参数
    imu_topic = rospy.get_param('~imu_topic', '/imu')
    position_x = rospy.get_param('~position_x', 1)
    position_y = rospy.get_param('~position_y', 1)
    position_z = rospy.get_param('~position_z', 0)
    world_frame_id = rospy.get_param('~world_frame_id', '/world')
    imu_frame_id = rospy.get_param('~imu_frame_id', '/imu')

    # 订阅 IMU 数据
    rospy.Subscriber(imu_topic, Imu, imu_callback)

    # 保持节点运行
    rospy.spin()