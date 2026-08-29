#!/usr/bin/env python3
"""
KCF 跟踪节点启动文件.

用法:
  ros2 launch wheeltec_robot_kcf_model kcf_tracker.launch.py
  ros2 launch wheeltec_robot_kcf_model kcf_tracker.launch.py \
      x1:=200 y1:=150 x2:=400 y2:=350 target_dist:=0.8
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions


def generate_launch_description():
    # ---- 初始跟踪框 ----
    x1_arg = DeclareLaunchArgument(
        'x1', default_value='290.0',
        description='初始跟踪框左上角 x (px)')
    y1_arg = DeclareLaunchArgument(
        'y1', default_value='160.0',
        description='初始跟踪框左上角 y (px)')
    x2_arg = DeclareLaunchArgument(
        'x2', default_value='350.0',
        description='初始跟踪框右下角 x (px)')
    y2_arg = DeclareLaunchArgument(
        'y2', default_value='200.0',
        description='初始跟踪框右下角 y (px)')

    # ---- 跟随控制 ----
    target_dist_arg = DeclareLaunchArgument(
        'target_dist', default_value='0.8',
        description='目标跟随距离 (m)')
    dist_deadzone_arg = DeclareLaunchArgument(
        'dist_deadzone', default_value='0.10',
        description='距离死区 (m)')
    angle_deadzone_arg = DeclareLaunchArgument(
        'angle_deadzone', default_value='0.03',
        description='角速度死区 (rad/s)')
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed', default_value='0.15',
        description='最大线速度 (m/s)')
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed', default_value='0.22',
        description='最大角速度 (rad/s)')

    # ---- 线性 PID ----
    linear_Kp_arg = DeclareLaunchArgument(
        'linear_Kp', default_value='0.5',
        description='线性 PID P 增益')
    linear_Ki_arg = DeclareLaunchArgument(
        'linear_Ki', default_value='0.0',
        description='线性 PID I 增益')
    linear_Kd_arg = DeclareLaunchArgument(
        'linear_Kd', default_value='1.5',
        description='线性 PID D 增益')

    # ---- 角速度 PID ----
    angular_Kp_arg = DeclareLaunchArgument(
        'angular_Kp', default_value='0.25',
        description='角速度 PID P 增益')
    angular_Ki_arg = DeclareLaunchArgument(
        'angular_Ki', default_value='0.0',
        description='角速度 PID I 增益')
    angular_Kd_arg = DeclareLaunchArgument(
        'angular_Kd', default_value='2.0',
        description='角速度 PID D 增益')

    # ---- 开关 ----
    enable_cmd_vel_arg = DeclareLaunchArgument(
        'enable_cmd_vel', default_value='true',
        description='是否直接发布 /cmd_vel 进行跟随')
    show_display_arg = DeclareLaunchArgument(
        'show_display', default_value='false',
        description='是否弹出本地实时显示窗口 (按 q 关闭)')

    # ---- 相机内参 ----
    camera_fx_arg = DeclareLaunchArgument(
        'camera_fx', default_value='606.0',
        description='相机 x 方向焦距 (px)')
    camera_fy_arg = DeclareLaunchArgument(
        'camera_fy', default_value='605.0',
        description='相机 y 方向焦距 (px)')
    camera_cx_arg = DeclareLaunchArgument(
        'camera_cx', default_value='321.0',
        description='光心 x 坐标 (px)')
    camera_cy_arg = DeclareLaunchArgument(
        'camera_cy', default_value='241.0',
        description='光心 y 坐标 (px)')

    # ---- YOLO 文件桥接 (conda 环境自动启动) ----
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='false',
        description='是否启用 YOLO 文件桥接修正 CSRT')
    yolo_python_arg = DeclareLaunchArgument(
        'yolo_python', default_value=os.path.expanduser('~/anaconda3/envs/wheeltec/bin/python3'),
        description='conda 环境的 python 路径')
    yolo_class_arg = DeclareLaunchArgument(
        'yolo_class', default_value='all',
        description='YOLO 检测目标 COCO 类别')
    yolo_interval_arg = DeclareLaunchArgument(
        'yolo_interval', default_value='30',
        description='YOLO 送帧间隔 (帧数, 30≈1s@30fps)')

    # YOLO 检测脚本 (conda 环境运行, 仅 use_yolo=true 时启动)
    pkg_share = get_package_share_directory('wheeltec_robot_kcf_model')
    yolo_script = os.path.join(pkg_share, 'yolo_detector.py')
    yolo_process = ExecuteProcess(
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('use_yolo'), '" == "true"'])),
        cmd=[LaunchConfiguration('yolo_python'), yolo_script,
             '--class', LaunchConfiguration('yolo_class')],
        output='screen',
    )

    # ---- KCF 跟踪节点 ----
    kcf_tracker_node = launch_ros.actions.Node(
        package='wheeltec_robot_kcf_model',
        executable='kcf_tracker',
        name='kcf_tracker_model',
        output='screen',
        parameters=[{
            'x1':               LaunchConfiguration('x1'),
            'y1':               LaunchConfiguration('y1'),
            'x2':               LaunchConfiguration('x2'),
            'y2':               LaunchConfiguration('y2'),
            'target_dist':      LaunchConfiguration('target_dist'),
            'dist_deadzone':    LaunchConfiguration('dist_deadzone'),
            'angle_deadzone':   LaunchConfiguration('angle_deadzone'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed':LaunchConfiguration('max_angular_speed'),
            'linear_Kp':        LaunchConfiguration('linear_Kp'),
            'linear_Ki':        LaunchConfiguration('linear_Ki'),
            'linear_Kd':        LaunchConfiguration('linear_Kd'),
            'angular_Kp':       LaunchConfiguration('angular_Kp'),
            'angular_Ki':       LaunchConfiguration('angular_Ki'),
            'angular_Kd':       LaunchConfiguration('angular_Kd'),
            'enable_cmd_vel':   LaunchConfiguration('enable_cmd_vel'),
            'show_display':     LaunchConfiguration('show_display'),
            'camera_fx':        LaunchConfiguration('camera_fx'),
            'camera_fy':        LaunchConfiguration('camera_fy'),
            'camera_cx':        LaunchConfiguration('camera_cx'),
            'camera_cy':        LaunchConfiguration('camera_cy'),
            'use_yolo':          LaunchConfiguration('use_yolo'),
            'yolo_interval':     LaunchConfiguration('yolo_interval'),
        }],
    )

    return LaunchDescription([
        # 初始跟踪框
        x1_arg, y1_arg, x2_arg, y2_arg,
        # 跟随控制
        target_dist_arg, dist_deadzone_arg, angle_deadzone_arg,
        max_linear_speed_arg, max_angular_speed_arg,
        # 线性 PID
        linear_Kp_arg, linear_Ki_arg, linear_Kd_arg,
        # 角速度 PID
        angular_Kp_arg, angular_Ki_arg, angular_Kd_arg,
        # 开关
        enable_cmd_vel_arg, show_display_arg,
        # 相机内参
        camera_fx_arg, camera_fy_arg, camera_cx_arg, camera_cy_arg,
        # YOLO
        use_yolo_arg, yolo_python_arg, yolo_class_arg, yolo_interval_arg,
        # 跟踪节点 + YOLO 进程
        kcf_tracker_node,
        yolo_process,
    ])
