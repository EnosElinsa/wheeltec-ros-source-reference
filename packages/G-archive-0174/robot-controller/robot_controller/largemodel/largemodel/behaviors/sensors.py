#!/usr/bin/env python3
"""传感器与进程工具 Mixin：传感器配置、健康检查、回调、子进程管理"""

import os
import time

import psutil
from std_msgs.msg import String
from .constants import normalize_angle


class SensorMixin:
    """传感器与工具方法，混入 StepActionServer"""

    def _check_all_sensors(self):
        """检查所有传感器话题是否有发布者。返回: (all_ok: bool, missing: list)
        开机后 startup_grace_period 秒内跳过检测，等待传感器驱动就绪。"""
        grace = self._safety.get('startup_grace_period', 10.0)
        if time.time() - self._node_start_time < grace:
            return True, []
        missing = []
        for key, cfg in self._sensors.items():
            topic = cfg.get('topic', '')
            label = cfg.get('label', key)
            if topic and not self.get_publishers_info_by_topic(topic):
                missing.append(label)
                self.get_logger().warn(f'传感器 {label} ({topic}) 无发布者')
        return len(missing) == 0, missing

    def _check_sensor_timer(self):
        """定期（5s）后台检查传感器话题是否在线"""
        all_ok, missing = self._check_all_sensors()
        if not all_ok:
            self._publish_status('sensor_warning', names='、'.join(missing))

    def wakeup_callback(self, msg):
        """打断回调：awake_flag=1 时设置打断标志"""
        if msg.data == 1:
            self.interrupt_flag = True
            # self.get_logger().info('收到打断信号')

    def distanceCallback(self, msg):
        """障碍物距离/角度回调（来自 /object_tracker/current_position）"""
        angle = msg.angle_x
        self.obstacle_angle = normalize_angle(angle)
        self.obstacle_dist = msg.distance

    def odom_callback(self, msg):
        """里程计回调：记录实际速度和位姿"""
        self.current_twist = msg.twist.twist
        self._current_odom_pose = msg.pose.pose
        self.last_odom_time = time.time()

    def _wait_subprocess(self, future, process, timeout=None):
        """等待 future 完成或进程退出或被打断。进程意外退出时自动解除阻塞。"""
        start_time = time.time()
        while not future.done():
            if self.interrupt_flag:
                break
            if process is not None and process.poll() is not None:
                self.get_logger().warn(f'子进程 pid={process.pid} 意外退出，返回码: {process.returncode}')
                if not future.done():
                    future.set_result(True)
                break
            if timeout is not None and (time.time() - start_time) > timeout:
                self.get_logger().warn('子进程等待超时')
                if not future.done():
                    future.set_result(True)
                break
            time.sleep(0.1)
        if process is not None:
            self._kill_process_tree(process.pid)

    @staticmethod
    def _kill_process_tree(pid):
        """杀死进程及其所有子进程"""
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()
        except psutil.NoSuchProcess:
            pass
