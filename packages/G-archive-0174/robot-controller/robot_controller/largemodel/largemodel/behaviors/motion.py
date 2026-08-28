#!/usr/bin/env python3
"""基础运动 Mixin：旋转、直行、速度控制 + 避障 + 卡住检测"""

import math
import time

from geometry_msgs.msg import Twist
from .constants import (
    normalize_angle, quat_to_yaw,
)


class MotionMixin:
    """基础运动方法，混入 StepActionServer"""

    def rotate(self, angle, angular_speed=1.5):
        """旋转 angle 度（正=左转，负=右转），ODO 闭环：转到目标角度即停"""
        angle = float(angle)
        angular_speed = float(angular_speed)
        direction = 1 if angle >= 0 else -1
        angle_rad = math.radians(abs(angle))
        max_duration = abs(angle_rad / angular_speed) + 0.8

        twist = Twist()
        twist.angular.z = abs(angular_speed) * direction
        odo_was_available = self.current_twist is not None
        interrupt_reason = self._execute_action(
            twist, durationtime=max_duration, target_angle_rad=angle_rad,
        )
        if interrupt_reason:
            self._set_last_result(True, interrupt_reason, silent=True)
        else:
            self._set_last_result(True, 'rotate_done', angle=abs(angle))
        if not odo_was_available:
            self._last_message += '（ODO不可用，开环运行）'
            self._publish_feedback_text(self._last_message)

    def go_straight(self, distance, speed=0.3):
        """直行 distance 米（正=前进，负=后退），ODO 闭环：到达目标距离即停"""
        distance = float(distance)
        speed = float(speed)
        direction = 1 if distance >= 0 else -1
        max_duration = abs(distance / speed) + 1.0

        twist = Twist()
        twist.linear.x = abs(speed) * direction
        twist.angular.z = self._safety.get('drift_compensation', 0.0) * direction
        odo_was_available = self.current_twist is not None
        interrupt_reason = self._execute_action(
            twist, durationtime=max_duration, target_distance=abs(distance),
        )
        if interrupt_reason:
            self._set_last_result(True, interrupt_reason, silent=True)
        else:
            self._set_last_result(True, 'go_straight_done', distance=abs(distance))
        if not odo_was_available:
            self._last_message += '（ODO不可用，开环运行）'
            self._publish_feedback_text(self._last_message)

    def set_cmdvel(self, linear_x, linear_y, angular_z, duration):
        """纯速度模式：按指定速度和时长发布 Twist，不做 ODO 闭环"""
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        interrupt_reason = self._execute_action(twist, durationtime=duration)
        if interrupt_reason:
            self._set_last_result(True, interrupt_reason, silent=True)
        else:
            self._set_last_result(True, 'set_cmdvel_done')

    def wait(self, duration):
        """等待 duration 秒，可被打断"""
        duration = float(duration)
        elapsed = 0.0
        while elapsed < duration:
            if self.interrupt_flag:
                break
            time.sleep(0.1)
            elapsed += 0.1
        self._set_last_result(True, 'wait_done', duration=duration)

    def _execute_action(self, twist, durationtime=3.0,
                        target_distance=None, target_angle_rad=None):
        """
        执行 twist 指令，期间检测打断、避障、卡住，并支持 ODO 闭环控制。
        durationtime: 最大持续时间（超时保护）
        target_distance: 目标线位移 (m)，odom 累计达到后提前停止
        target_angle_rad: 目标角位移 (rad)，odom 累计达到后提前停止
        返回: None 正常完成，'obstacle_stop' 遇障停车，'robot_stuck' 卡住停车
        """
        start_time = time.time()
        obstacle_count = 0
        stuck_count = 0
        odo_needed = target_distance is not None or target_angle_rad is not None
        odo_warned = False
        initial_pose = None
        initial_yaw = 0.0

        while (time.time() - start_time) < durationtime:
            # 避障检测
            if self._obstacle_in_path(twist, self.obstacle_dist, self.obstacle_angle):
                obstacle_count += 1
                if obstacle_count >= 3:
                    self.stop()
                    self._publish_status('obstacle_stop')
                    return 'obstacle_stop'
            else:
                obstacle_count = 0

            # 卡住检测：对比指令速度与实际 odom 速度
            if self._robot_is_stuck(twist):
                stuck_count += 1
                if stuck_count >= int(self._safety.get('stuck_duration', 1.0) / 0.1):
                    self.stop()
                    self._publish_status('robot_stuck')
                    return 'robot_stuck'
            else:
                stuck_count = 0

            # ODO 闭环不可用告警（运动开始 2 秒后仍未收到 odom 数据）
            if odo_needed and not odo_warned and self.current_twist is None:
                if time.time() - start_time > 2.0:
                    self._publish_status('sensor_warning', names='里程计')
                    odo_warned = True

            # ODO 闭环：用位姿差算实际位移/角度，到达目标提前停止
            if self._current_odom_pose is not None and odo_needed:
                if initial_pose is None:
                    initial_pose = self._current_odom_pose
                    initial_yaw = quat_to_yaw(initial_pose.orientation)

                current_pose = self._current_odom_pose
                dx = current_pose.position.x - initial_pose.position.x
                dy = current_pose.position.y - initial_pose.position.y
                current_dist = math.sqrt(dx * dx + dy * dy)
                current_yaw = quat_to_yaw(current_pose.orientation)
                current_angle = abs(normalize_angle(current_yaw - initial_yaw))

                if target_distance is not None and current_dist >= target_distance:
                    self.stop()
                    return None
                if target_angle_rad is not None and current_angle >= target_angle_rad:
                    self.stop()
                    return None

            if self.interrupt_flag:
                self.stop()
                return None
            self.publisher.publish(twist)
            time.sleep(0.1)
        self.stop()
        return None

    def _obstacle_in_path(self, cmd, d, ang):
        """避障判断：根据当前速度指令和障碍物距离/角度，判断是否处于行驶路径上。"""
        predict_time = self._safety.get('predict_time', 1.5)
        robot_width = self._safety.get('robot_width', 0.3)
        lidar_range = self._safety.get('lidar_range', 0.8)

        if d > lidar_range:
            return False
        v = cmd.linear.x
        w = cmd.angular.z
        if abs(w) < 1e-3:
            along = v * predict_time
            across = robot_width
        else:
            r = v / (w + 1e-3)
            along = r * math.sin(w * predict_time)
            across = robot_width + abs(r * (1 - math.cos(w * predict_time)))
        x = d * math.cos(ang)
        y = d * math.sin(ang)
        x_in = (0 <= x <= along) if along >= 0 else (along <= x <= 0)
        y_in = abs(y) <= across
        return x_in and y_in

    def _robot_is_stuck(self, cmd):
        """检测机器人是否卡住：对比指令速度与 odom 实际速度。"""
        if self.current_twist is None:
            return False
        if time.time() - self.last_odom_time > 0.5:
            return False

        min_cmd_speed = self._safety.get('min_cmd_speed', 0.05)
        stuck_threshold = self._safety.get('stuck_speed_threshold', 0.03)

        cmd_linear = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
        cmd_angular = abs(cmd.angular.z)

        if cmd_linear >= cmd_angular and cmd_linear > min_cmd_speed:
            actual = math.sqrt(
                self.current_twist.linear.x ** 2 + self.current_twist.linear.y ** 2
            )
            return actual < stuck_threshold
        elif cmd_angular > min_cmd_speed:
            actual = abs(self.current_twist.angular.z)
            return actual < stuck_threshold

        return False

    def stop(self):
        """停止机器人（静默，不发布状态）。也可作为 AI 动作调用。"""
        twist = Twist()
        self.publisher.publish(twist)
        self._set_last_result(True, 'stop_done', silent=True)
