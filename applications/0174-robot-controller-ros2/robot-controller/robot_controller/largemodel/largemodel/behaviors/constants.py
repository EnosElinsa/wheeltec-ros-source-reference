#!/usr/bin/env python3
"""机器人运动参数默认值与数学工具函数。

安全参数的实际值从 param.yaml 读取，此文件仅保留默认值作为 fallback
和 normalize_angle / quat_to_yaw 等纯数学函数。
"""

import math

# ---------- 安全参数默认值（运行时由 param.yaml 覆盖）----------
DEFAULT_SAFETY = {
    'predict_time': 1.5,
    'robot_width': 0.3,
    'lidar_range': 0.8,
    'stuck_speed_threshold': 0.03,
    'stuck_duration': 1.0,
    'min_cmd_speed': 0.05,
    'drift_compensation': 0.0,
    'max_linear_velocity': 1.0,
    'max_angular_velocity': 1.5,
    'startup_grace_period': 10.0,  # 开机后等待传感器就绪的时间 (秒)
}


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(q) -> float:
    """从 geometry_msgs/Quaternion 提取 yaw 角"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)
