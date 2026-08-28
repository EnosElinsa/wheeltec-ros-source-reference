#!/usr/bin/env python3
"""
单步动作 HTTP 桥梁 — OpenClaw 通过 HTTP POST 逐动作发送，等待反馈后决定下一步。
端口 9090，零外部依赖（Python 标准库 + rclpy）。

FIFO 负责通信进程管理：
  - 存活检测：写端开着 = bridge 活着，EOF = bridge 下线
  - 心跳检测：每 5s 写 "ping\\n"，BrokenPipeError 表示 OpenClaw 读端断开
  - 异步事件：obstacle_stop / robot_stuck / sensor_warning

语音注入由 bridge 直接 POST OpenClaw API，不走 FIFO。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from interfaces.action import StepAction
from std_msgs.msg import String
import atexit
import json
import os
import queue
import time
import threading
import urllib.request
import inspect
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler


class ActionBridge(Node):
    """监听 HTTP 请求，每次转发一个动作到 action_service"""

    def __init__(self):
        super().__init__('action_bridge')
        self._client = ActionClient(self, StepAction, 'step_action')
        self._latest_status = None
        self._latest_feedback = None
        self._latest_image_path = None

        self.create_subscription(String, 'actionstatus', self._on_status, 10)
        self.create_subscription(String, 'feedback_words', self._on_feedback, 10)
        self.create_subscription(String, 'voice_words', self._on_voice, 10)
        self.text_pub = self.create_publisher(String, 'feedback_words', 10)

        self._latest_voice_command = None
        self._voice_timestamp = 0.0

        # --- 动作执行状态追踪 ---
        self._action_running = False
        self._current_action = None
        self._sensor_warnings = []   # 最近传感器告警列表

        # --- OpenClaw 连接状态 + latched publisher ---
        self._openclaw_connected = False
        self._openclaw_client_id = None
        self.status_pub = self.create_publisher(
            String, 'bridge_status',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_bridge_status()

        # --- FIFO 命名管道：双端事件通道 ---
        self._ready_fifo = '/tmp/robot_bridge_ready'
        try:
            os.unlink(self._ready_fifo)
        except FileNotFoundError:
            pass
        os.mkfifo(self._ready_fifo)
        self._fifo_wake = threading.Event()    # 有新事件时唤醒 daemon
        self._pending_event = None              # 待发送的事件类型
        self._event_lock = threading.Lock()     # 保护 _pending_event
        self._fifo_writer = None  # 写端 fd，由 daemon 线程持有
        threading.Thread(target=self._fifo_daemon, daemon=True).start()
        atexit.register(self._cleanup_fifo)
        self.get_logger().info(f'FIFO 事件通道已创建: {self._ready_fifo}')

        # 加载 OpenClaw Gateway token（用于语音直接注入 API）
        self._openclaw_token = self._load_openclaw_token()
        self._openclaw_api = 'http://127.0.0.1:18789/v1/chat/completions'
        self._voice_queue = queue.Queue()
        self._step_lock = threading.Lock()
        threading.Thread(target=self._voice_sender, daemon=True).start()

        # --- Speak 客户端：调用 action_service 的统一队列 ---
        from interfaces.srv import SetString
        self._speak_client = self.create_client(SetString, '/action_service/speak')

        self.get_logger().info('单步 ActionBridge 就绪')

    # ======================== Speak 接口 ========================

    def _call_speak_service(self, text):
        """同步调用 action_service 的 speak 队列，返回 (ok, queued_count)。"""
        if not self._speak_client.wait_for_service(timeout_sec=2.0):
            return False, 0
        from interfaces.srv import SetString
        req = SetString.Request()
        req.data = text
        done = threading.Event()
        result = [None]

        def _cb(future):
            result[0] = future.result()
            done.set()

        future = self._speak_client.call_async(req)
        future.add_done_callback(_cb)
        done.wait(timeout=5.0)
        if result[0] is None:
            return False, 0
        return result[0].success, int(result[0].message) if result[0].message.isdigit() else 0

    def speak(self, text):
        """OpenClaw 调用：转发到 action_service 的统一 speak 队列。"""
        self._latest_feedback = text
        ok, queued = self._call_speak_service(text)
        if ok:
            return {'ok': True, 'queued': queued}
        else:
            self.get_logger().error('Speak service 不可用')
            return {'ok': False, 'error': 'speak service unavailable'}

    # ======================== Bridge 状态管理 ========================

    def _publish_bridge_status(self):
        msg = String()
        msg.data = json.dumps({
            'bridge_up': True,
            'openclaw_connected': self._openclaw_connected,
            'client_id': self._openclaw_client_id,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def register(self, client_id):
        self._openclaw_connected = True
        self._openclaw_client_id = client_id
        self._publish_bridge_status()
        self.get_logger().info(f'OpenClaw 已注册 (client_id={client_id})')
        return {'ok': True, 'client_id': client_id}

    def unregister(self):
        self._openclaw_connected = False
        self._openclaw_client_id = None
        self._publish_bridge_status()
        self.get_logger().info('OpenClaw 已注销，拒绝新动作')
        return {'ok': True}

    # ======================== FIFO 守护线程 ========================

    def _fifo_daemon(self):
        """
        FIFO 通信进程管理：
          - "ready\\n"           — 初始就绪信号
          - "ping\\n"             — 每 5s 心跳（BrokenPipeError → watchdog 已断开）
          - "obstacle_stop\\n"   — 避障停车
          - "robot_stuck\\n"     — 机器人卡住
          - "sensor_warning\\n"  — 传感器异常
        语音已改由 bridge 直接注入 OpenClaw API，不走 FIFO。
        """
        while rclpy.ok():
            try:
                with open(self._ready_fifo, 'w') as f:
                    self._fifo_writer = f
                    f.write('ready\n')
                    f.flush()
                    self.get_logger().info('FIFO 连接已建立，发送 ready 信号')

                    while rclpy.ok():
                        triggered = self._fifo_wake.wait(timeout=5)

                        if triggered:
                            self._fifo_wake.clear()
                            with self._event_lock:
                                event_type = self._pending_event
                                self._pending_event = None
                            if event_type:
                                try:
                                    f.write(f'{event_type}\n')
                                    f.flush()
                                except BrokenPipeError:
                                    self.get_logger().warn(f'FIFO 写端断开 ({event_type})')
                                    break
                            continue

                        # 超时 → 写心跳 ping
                        try:
                            f.write('ping\n')
                            f.flush()
                        except BrokenPipeError:
                            self.get_logger().warn('FIFO BrokenPipeError — watchdog 读端已关闭，自动注销')
                            if self._openclaw_connected:
                                self.unregister()
                            break

            except Exception as e:
                self.get_logger().warn(f'FIFO daemon 异常 (将重试): {e}')
            finally:
                self._fifo_writer = None
                time.sleep(1)

    def _push_fifo_event(self, event_type):
        """唤醒 FIFO daemon 即时写入指定事件类型"""
        with self._event_lock:
            self._pending_event = event_type
        self._fifo_wake.set()

    def _cleanup_fifo(self):
        try:
            os.unlink(self._ready_fifo)
        except FileNotFoundError:
            pass

    # ======================== 语音直接注入 OpenClaw ========================

    @staticmethod
    def _load_openclaw_token():
        try:
            cfg_path = os.path.expanduser('~/.openclaw/openclaw.json')
            with open(cfg_path) as f:
                cfg = json.load(f)
            return cfg.get('gateway', {}).get('auth', {}).get('token', '')
        except Exception:
            return ''

    def _inject_voice(self, text):
        """将语音文本放入队列，由单一线程消费注入 OpenClaw"""
        if not self._openclaw_token:
            self.get_logger().warn('未找到 OpenClaw token，无法注入语音')
            return
        self._voice_queue.put(text)

    def _voice_sender(self):
        """单一线程消费语音队列，逐个 POST OpenClaw API"""
        while rclpy.ok():
            try:
                text = self._voice_queue.get(timeout=1)
            except queue.Empty:
                continue
            payload = json.dumps({
                'model': 'openclaw/wheeltec_robot',
                'user': 'voice',
                'messages': [{'role': 'user', 'content': text}],
            }).encode('utf-8')
            req = urllib.request.Request(self._openclaw_api, data=payload, headers={
                'Authorization': f'Bearer {self._openclaw_token}',
                'Content-Type': 'application/json',
            }, method='POST')
            try:
                urllib.request.urlopen(req, timeout=120)
            except Exception as e:
                self.get_logger().error(f'语音注入 OpenClaw 失败: {e}')

    # ======================== ROS 话题回调 ========================

    def _on_voice(self, msg):
        """语音识别回调：异步注入 OpenClaw，不阻塞 ROS"""
        text = msg.data.strip()
        if not text:
            return
        self._latest_voice_command = text
        self._voice_timestamp = time.time()
        self._inject_voice(text)
        self.get_logger().info(f'收到语音指令: {text}')

    def _on_status(self, msg):
        self._latest_status = msg.data
        status = msg.data
        if status == 'sensor_warning':
            self._sensor_warnings.append(status)
            if len(self._sensor_warnings) > 10:
                self._sensor_warnings.pop(0)
        if status in ('action_started', 'finish', 'obstacle_stop', 'robot_stuck', 'sensor_warning') or \
           status.endswith(('_done', '_error', '_failed', '_cancelled')):
            self._push_fifo_event(status)

    def _on_feedback(self, msg):
        self._latest_feedback = msg.data

    # ======================== 执行单步动作 ========================

    def step(self, action, timeout=60.0):
        """
        执行单步动作，同步等待结果。
        使用 threading.Event 等待，避免在 HTTP 线程 spin executor 导致回调死锁。
        """
        self._action_running = True
        self._current_action = action
        try:
            with self._step_lock:
                return self._step_impl(action, timeout)
        finally:
            self._action_running = False
            self._current_action = None

    def _step_impl(self, action, timeout):
        if not self._client.wait_for_server(timeout_sec=5.0):
            return {'success': False, 'error': 'action_service 不可用', 'status': 'error'}

        goal = StepAction.Goal()
        goal.action = action
        self._latest_status = None
        self._latest_feedback = None

        # 局部变量 + 闭包，避免实例变量被后续 step 覆盖导致竞态
        step_done = threading.Event()
        step_result = [None]  # 容器供闭包写入
        send_done = threading.Event()

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: send_done.set())

        if not send_done.wait(timeout=10.0):
            return {'success': False, 'error': '发送超时', 'status': 'error'}

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return {'success': False, 'error': '服务端忙碌（上一动作未完成）', 'status': 'busy'}

        def _on_step_done(future):
            if step_done.is_set():
                return
            response = future.result()
            result = response.result
            image_path = result.image_path or None
            message = getattr(result, 'message', '') or ''
            if image_path:
                self._latest_image_path = image_path
            step_result[0] = {
                'success': result.success,
                'status': result.status,
                'message': message,
                'image_path': image_path,
                'feedback': self._latest_feedback,
            }
            step_done.set()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(_on_step_done)

        if not step_done.wait(timeout=timeout):
            return {'success': False, 'error': '执行超时', 'status': 'timeout'}

        return step_result[0]


# ======================== 动作发现工具 ========================

def _get_available_actions():
    """扫描所有 Mixin 类，返回 AI 可调用的动作清单。
    与 action_service 的 get_available_actions() 逻辑一致，
    但以独立函数形式供 bridge HTTP 端点直接调用。"""
    from largemodel.behaviors.motion import MotionMixin
    from largemodel.behaviors.navigation import NavigationMixin
    from largemodel.behaviors.vision import VisionMixin
    from largemodel.behaviors.sensors import SensorMixin

    UTILITY_METHODS = {'load_target_points', 'save_single_image'}

    domain_map = {
        MotionMixin: 'motion',
        NavigationMixin: 'navigation',
        VisionMixin: 'vision',
        SensorMixin: 'sensor',
    }

    actions = []
    for mixin, domain in domain_map.items():
        for name, method in inspect.getmembers(mixin, inspect.isfunction):
            if name.startswith('_'):
                continue
            if name in UTILITY_METHODS:
                continue
            doc = inspect.getdoc(method) or ''
            sig = inspect.signature(method)
            params = [{'name': p.name} for p in sig.parameters.values() if p.name != 'self']
            actions.append({
                'name': name,
                'description': doc.split('\n')[0] if doc else '',
                'params': params,
                'domain': domain,
            })
    return actions


# ======================== HTTP Handler ========================

class BridgeHandler(BaseHTTPRequestHandler):
    bridge = None

    def _json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except BrokenPipeError:
            pass

    def _read_json_body(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            return json.loads(self.rfile.read(length))
        except (ValueError, json.JSONDecodeError):
            return None

    def do_POST(self):
        if self.path == '/step':
            body = self._read_json_body()
            if body is None:
                self._json({'success': False, 'error': '无效 JSON'}, 400)
                return
            action = body.get('action', '')
            timeout = body.get('timeout', 60)
            if not action:
                self._json({'success': False, 'error': '缺少 action 字段'}, 400)
                return
            result = self.bridge.step(action, timeout)
            self._json(result)

        elif self.path == '/speak':
            body = self._read_json_body()
            if body is None:
                self._json({'ok': False, 'error': '无效 JSON'}, 400)
                return
            text = body.get('text', '')
            if not text:
                self._json({'ok': False, 'error': '缺少 text 字段'}, 400)
                return
            result = self.bridge.speak(text)
            self._json(result)

        elif self.path == '/register':
            body = self._read_json_body()
            if body is None:
                self._json({'ok': False, 'error': '无效 JSON'}, 400)
                return
            client_id = body.get('client_id', 'unknown')
            result = self.bridge.register(client_id)
            self._json(result)

        elif self.path == '/unregister':
            result = self.bridge.unregister()
            self._json(result)

        else:
            self._json({'error': 'not found'}, 404)

    def do_GET(self):
        if self.path == '/health':
            self._json({
                'ok': True,
                'status': self.bridge._latest_status,
                'openclaw_connected': self.bridge._openclaw_connected,
            })
        elif self.path == '/status':
            has_warning = len(self.bridge._sensor_warnings) > 0
            self._json({
                'action_status': self.bridge._latest_status,
                'feedback': self.bridge._latest_feedback,
                'openclaw_connected': self.bridge._openclaw_connected,
                'action_running': self.bridge._action_running,
                'current_action': self.bridge._current_action,
                'sensor_ok': not has_warning,
            })
        elif self.path == '/actions':
            self._json(_get_available_actions())
        elif self.path == '/voice':
            vc = self.bridge._latest_voice_command
            ts = self.bridge._voice_timestamp
            self._json({
                'command': vc,
                'timestamp': ts,
                'stale': (time.time() - ts) > 30 if ts else True,
            })
        elif self.path.startswith('/image'):
            image_path = self.bridge._latest_image_path
            if not image_path:
                self._json({'error': '暂无图片'}, 404)
                return
            try:
                with open(image_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception:
                self._json({'error': '图片不可用'}, 404)
        else:
            self._json({'error': 'not found'}, 404)

    def log_message(self, format, *args):
        pass


# ======================== 主函数 ========================

def main():
    rclpy.init(args=None)
    bridge = ActionBridge()
    BridgeHandler.bridge = bridge

    server = ThreadingHTTPServer(('0.0.0.0', 9090), BridgeHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    bridge.get_logger().info('单步 ActionBridge HTTP 服务已启动 (0.0.0.0:9090, ThreadingHTTPServer)')

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        bridge.get_logger().info('ActionBridge 关闭中')
    finally:
        executor.shutdown()
        server.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
