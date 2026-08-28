#!/usr/bin/env python3
"""视觉跟随 Mixin：拍照、KCF、巡线、颜色跟随、雷达跟随"""

import os
import time
import subprocess
import cv2

from cv_bridge import CvBridge
from concurrent.futures import Future


class VisionMixin:
    """视觉与跟随方法，混入 StepActionServer"""

    image_save_path: str = os.path.expanduser('~/.openclaw/agents/wheeltec_robot/seewhat_snapshot.png')

    def seewhat(self):
        """拍照并保存图片。OpenClaw 自行读取图片文件分析。"""
        if self.save_single_image():
            self._set_last_result(True, 'seewhat_done', image_path=self.image_save_path)
        else:
            self._set_last_result(False, 'action_error', error='相机未就绪，请稍后重试')

    # ======================== 跟随进程管理 ========================

    _FOLLOWER_FUTURES = ['visual_follower_future', 'laser_follower_future',
                         'line_follower_future', 'KCF_follow_future']

    def _start_follower(self, future_attr, main_proc, cleanup_pids=None):
        """统一管理跟随进程：等待 -> 清理 -> 停止。异常由调用方处理。"""
        self._wait_subprocess(getattr(self, future_attr), main_proc)
        if cleanup_pids:
            for pid in cleanup_pids:
                self._kill_process_tree(pid)
        self.stop()
        self._set_last_result(True, 'stop_follow_done', silent=True)

    def _stop_all_followers(self):
        for attr in self._FOLLOWER_FUTURES:
            future = getattr(self, attr)
            if future and not future.done():
                future.set_result(True)

    # ======================== 各跟随操作 ========================

    COLOR_MAP = {'red': 0, 'green': 1, 'blue': 2, 'yellow': 3}

    def KCF_follow(self, x1=0.0, y1=0.0, x2=0.0, y2=0.0):
        """KCF 视觉跟踪。参数为浮点归一化坐标（0.0-1000.0），自动换算为 640x480。"""
        x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
        if x1 == y1 == x2 == y2 == 0.0:
            self.seewhat()
            return
        px1, py1 = x1 * 640 / 1000, y1 * 480 / 1000
        px2, py2 = x2 * 640 / 1000, y2 * 480 / 1000
        self.get_logger().info(f'KCF_follow: x1:{px1:.0f}; y1:{py1:.0f}; x2:{px2:.0f}; y2:{py2:.0f}')
        self.KCF_follow_future = Future()
        p = subprocess.Popen(['ros2', 'run', 'wheeltec_robot_kcf_model', 'kcf_tracker',
            '--ros-args', '-p', f'x1:={px1:.1f}',
            '-p', f'y1:={py1:.1f}',
            '-p', f'x2:={px2:.1f}',
            '-p', f'y2:={py2:.1f}'])
        self._start_follower('KCF_follow_future', p)

    def visual_follower(self, color='red'):
        color = color.strip('"\'')
        target = self.COLOR_MAP.get(color, 0)
        self.visual_follower_future = Future()
        p1 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualtracker',
            '--ros-args', '-p', f'target_color:={target}'])
        p2 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualfollow'])
        self._start_follower('visual_follower_future', p1, cleanup_pids=[p2.pid])

    def line_follower(self, color='red'):
        color = color.strip('"\'')
        target = self.COLOR_MAP.get(color, 0)
        self.line_follower_future = Future()
        p = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'line_follow_model',
            '--ros-args', '-p', f'target_color:={target}'])
        self._start_follower('line_follower_future', p)

    def laser_follower(self):
        self.laser_follower_future = Future()
        p = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'laserfollower'])
        self._start_follower('laser_follower_future', p)

    def stop_follow(self):
        self._stop_all_followers()
        self._set_last_result(True, 'stop_follow_done', silent=True)

    def save_single_image(self):
        """保存一张相机图像到 image_save_path，返回是否成功"""
        with self._state_lock:
            self.IS_SAVING = True
        time.sleep(0.1)
        success = False
        with self._state_lock:
            if self.image_msg is None:
                self.get_logger().warning('尚未接收到图像，无法保存')
            else:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, 'bgr8')
                    # 存档上一张到 latest_view.png
                    archive_path = os.path.join(os.path.dirname(self.image_save_path), 'latest_view.png')
                    if os.path.exists(self.image_save_path):
                        os.rename(self.image_save_path, archive_path)
                    if not cv2.imwrite(self.image_save_path, cv_image):
                        self.get_logger().error(f'图像写入失败: {self.image_save_path}')
                    else:
                        self.get_logger().info(f'图像已保存: {self.image_save_path}')
                        success = True
                except Exception as e:
                    self.get_logger().error(f'保存图像失败: {e}')
            self.IS_SAVING = False
        return success

    def image_callback(self, msg):
        """相机图像回调"""
        with self._state_lock:
            if not self.IS_SAVING:
                self.image_msg = msg

    def finishtask(self):
        """结束当前任务，发布 finish 状态"""
        self._set_last_result(True, 'finish')
