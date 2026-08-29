#!/usr/bin/env python3
"""语音播报 Mixin — 统一 Speak 队列，L1/L2 消息串行播放"""

import queue
import threading
import time

import rclpy
from std_msgs.msg import String


class SpeakMixin:
    """统一 Speak 队列，混入 StepActionServer。

    所有 TTS 消息（L1 自动 + L2 OpenClaw 远程）都经 _enqueue_speak() 入队，
    worker 线程逐条发布到 feedback_words，估算播放时长后发下一条。
    """

    def init_speak_queue(self):
        """由 init_ros_communication 调用，初始化队列 + worker + service。"""
        self._speak_queue = queue.Queue()
        self._speaking = False
        threading.Thread(target=self._speak_worker, daemon=True).start()

        from interfaces.srv import SetString
        self._speak_srv = self.create_service(SetString, '~/speak', self._on_speak_service)
        self.get_logger().info('Speak 队列已初始化')

    def _enqueue_speak(self, text):
        """L1/L2 统一的语音入口。所有 TTS 消息都经此队列串行播放。"""
        if not text:
            return
        self._speak_queue.put(text)

    def _on_speak_service(self, request, response):
        """L2 OpenClaw TTS：HTTP /speak → bridge → 此 ROS2 service → 入队。"""
        text = request.data.strip()
        if not text:
            response.success = False
            response.message = 'empty'
            return response
        self._enqueue_speak(text)
        response.success = True
        response.message = str(self._speak_queue.qsize())
        return response

    def _speak_worker(self):
        """从队列取消息，逐条发布到 feedback_words。
        根据文本长度估算播放时间，等上一条播完再发下一条。"""
        CHARS_PER_SEC = 4.0        # 中文 TTS 约 4 字/秒
        MIN_GAP = 1.5              # 最小间隔（秒）

        while rclpy.ok():
            try:
                text = self._speak_queue.get(timeout=1)
            except queue.Empty:
                continue

            self._speaking = True
            msg = String()
            msg.data = text
            self.text_pub.publish(msg)
            self.get_logger().info(f'TTS 播放: {text}')

            duration = max(len(text) / CHARS_PER_SEC, MIN_GAP)
            time.sleep(duration)
            self._speaking = False
            self._speak_queue.task_done()
