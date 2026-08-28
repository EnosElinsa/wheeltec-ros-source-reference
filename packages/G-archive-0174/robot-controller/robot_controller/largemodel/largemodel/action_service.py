#!/usr/bin/env python3
"""
单步动作服务器 —— 每次接收一个动作，执行后返回详细反馈。
功能按模块拆分：motion / navigation / vision / sensors
"""

import json
import os
import re
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from interfaces.action import StepAction
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turn_on_wheeltec_robot.msg import Position
from rclpy.qos import qos_profile_sensor_data
from concurrent.futures import Future

from .behaviors.motion import MotionMixin
from .behaviors.navigation import NavigationMixin
from .behaviors.vision import VisionMixin
from .behaviors.sensors import SensorMixin
from .behaviors.speak import SpeakMixin
from .behaviors.constants import DEFAULT_SAFETY


class StepActionServer(MotionMixin, NavigationMixin, VisionMixin, SensorMixin, SpeakMixin, Node):
    """
    单步动作服务器。
    每次通过 StepAction 接收一个 action 字符串，执行后返回结果。
    方法分散在 MotionMixin / NavigationMixin / VisionMixin / SensorMixin 中。
    """

    def __init__(self):
        Node.__init__(self, 'action_service')
        self._node_start_time = time.time()
        self.init_param_config()
        self.init_ros_communication()
        self.init_navigation_client()
        self.get_logger().info('单步 ActionService 已启动')

    # ======================== 参数 & ROS 通信初始化 ========================

    def init_param_config(self):
        pkg_share = get_package_share_directory('largemodel')
        self.map_mapping_config = os.path.join(pkg_share, 'config', 'map_mapping.yaml')

        self.declare_parameter('Speed_topic', '/cmd_vel')
        self.declare_parameter('text_chat_mode', False)

        self.Speed_topic = self.get_parameter('Speed_topic').get_parameter_value().string_value
        self.text_chat_mode = self.get_parameter('text_chat_mode').get_parameter_value().bool_value

        # ROS 2 Humble 不支持 dict 参数，直接从 param.yaml 读取
        defaults = {
            'camera': {'topic': '/camera/color/image_raw', 'label': '相机', 'critical': True},
            'lidar': {'topic': '/scan', 'label': '雷达', 'critical': True},
            'odom': {'topic': '/odom', 'label': '里程计', 'critical': True},
        }
        try:
            param_file = os.path.join(pkg_share, 'config', 'param.yaml')
            with open(param_file) as f:
                cfg = yaml.safe_load(f)
            sensors = cfg.get('action_service', {}).get('ros__parameters', {}).get('sensors')
            if sensors:
                self._sensors = sensors
            else:
                self._sensors = defaults
        except Exception:
            self._sensors = defaults

        # 安全参数：从 param.yaml 读取，fallback 到 DEFAULT_SAFETY
        try:
            param_file = os.path.join(pkg_share, 'config', 'param.yaml')
            with open(param_file) as f:
                cfg = yaml.safe_load(f)
            safety_cfg = cfg.get('action_service', {}).get('ros__parameters', {}).get('safety')
            if safety_cfg:
                self._safety = safety_cfg
            else:
                self._safety = dict(DEFAULT_SAFETY)
        except Exception:
            self._safety = dict(DEFAULT_SAFETY)

        self.image_topic = self._sensors.get('camera', {}).get('topic', '/camera/color/image_raw')

        self.pkg_path = pkg_share
        self.image_save_path = os.path.join(
            os.path.expanduser('~/.openclaw/agents/wheeltec_robot'), 'seewhat_snapshot.png')
        os.makedirs(os.path.dirname(self.image_save_path), exist_ok=True)

        # Future 对象
        self.visual_follower_future = Future()
        self.laser_follower_future = Future()
        self.line_follower_future = Future()
        self.KCF_follow_future = Future()
        self.navigation_future = Future()
        self.slam_future = Future()

        # 状态标志
        self._state_lock = threading.Lock()
        self.interrupt_flag = False
        self.action_running = False
        self.IS_SAVING = False
        self.nav_runing = False
        self.nav_status = False
        self.navigation_finish_flag = False
        self.goal_handle = None

        # --- Bridge / OpenClaw 连接状态 ---
        self._openclaw_connected = False

        # 障碍物信息
        self.obstacle_angle = 0.0
        self.obstacle_dist = 0.0

        # 里程计反馈
        self.current_twist = None
        self.last_odom_time = 0.0
        self._current_odom_pose = None

        # 图像处理
        self.image_msg = None
        self.bridge = CvBridge()

        # 用户反馈消息模板
        self.feedback_dict = {
            'navigation_start': '开始导航啦，冲冲冲！',
            'navigation_done': '到达{point_name}啦，任务完成！',
            'navigation_failed': '导航失败了，可能是目标点{point_name}不存在',
            'navigation_cancelled': '导航被取消啦',
            'navigation_rejected': '导航请求被拒绝，可能需要先启动导航功能',
            'get_current_pose_success': '已记录当前位置{name}',
            'get_current_pose_failed': '位置记录失败，请重新定位',
            'set_description_done': '已更新{symbol}的描述信息',
            'set_description_failed': '描述更新失败，请确认位置{symbol}存在',
            'action_started': '收到，马上行动～',
            'visual_follower_started': '开始颜色跟随～',
            'line_follower_started': '开始巡线～',
            'laser_follower_started': '开始雷达跟随～',
            'KCF_follow_started': '开始视觉跟踪～',
            'wait_done': '等待{duration}秒完成',
            'set_cmdvel_done': '速度控制完成',
            'seewhat_done': '拍照完成',
            'rotate_done': '旋转{angle}度完成',
            'go_straight_done': '直行{distance}米完成',
            'slam_start': 'SLAM 建图启动完成',
            'slam_stop': 'SLAM 建图结束，地图已保存',
            'navigation_start_done': '导航功能启动完成',
            'navigation_start_timeout': '导航服务启动超时，请稍后重试',
            'navigation_stop_done': '导航功能已停止',
            'stop_done': '已停止',
            'stop_follow_done': '跟随已停止',
            'finish': '任务结束啦',
            'obstacle_stop': '遇到障碍物，停止移动',
            'robot_stuck': '机器人可能卡住了，实际未移动，已停止',
            'action_not_found': '动作函数不存在，无法执行',
            'action_error': '执行出错：{error}',
            'action_parse_error': '无法解析动作指令',
            'sensor_warning': '传感器警告：{names}无数据，请检查硬件连接',
        }

    def init_ros_communication(self):
        """初始化 ROS 通信对象"""
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)

        self._action_server = ActionServer(
            self, StepAction, 'step_action',
            execute_callback=self.execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.actionstatus_pub = self.create_publisher(String, 'actionstatus', 3)
        self.text_pub = self.create_publisher(String, 'feedback_words', 1)

        # --- 统一 Speak 队列（L1+L2 共享入口）---
        self.init_speak_queue()


        self.wakeup = self.create_subscription(Int8, 'awake_flag', self.wakeup_callback, 1)

        self.position_sub = self.create_subscription(
            Position, '/object_tracker/current_position',
            self.distanceCallback, qos_profile=qos_profile_sensor_data,
        )

        odom_topic = self._sensors.get('odom', {}).get('topic', '/odom')
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 2,
        )

        self._check_timer = self.create_timer(5.0, self._check_sensor_timer)

        # 订阅 Bridge 状态（latched），获取 OpenClaw 连接状态
        self.bridge_status_sub = self.create_subscription(
            String, 'bridge_status', self._on_bridge_status,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def init_navigation_client(self):
        """初始化导航客户端"""
        self.load_target_points()
        self.navclient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()

    # ======================== 动作自省 ========================

    def get_available_actions(self):
        """自动发现 StepActionServer 上所有 Mixin 提供的公开动作方法。

        遍历 MotionMixin / NavigationMixin / VisionMixin / SensorMixin，
        返回每个公开方法的名称、描述、参数和所属功能域，供 AI 动态获取
        最新的动作能力清单。
        """
        import inspect
        from .behaviors.motion import MotionMixin
        from .behaviors.navigation import NavigationMixin
        from .behaviors.vision import VisionMixin
        from .behaviors.sensors import SensorMixin

        # 工具方法（公开但不是 AI 可调用的动作）
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

    # ======================== ActionServer 回调 ========================

    def _goal_callback(self, goal_request):
        with self._state_lock:
            if self.action_running:
                self.get_logger().warn('拒绝新目标：上一动作仍在执行')
                return GoalResponse.REJECT
            if not self._openclaw_connected:
                self.get_logger().warn('拒绝新目标：OpenClaw 尚未注册')
                return GoalResponse.REJECT
            self.action_running = True
        func_name = goal_request.action.split('(')[0] if '(' in goal_request.action else ''
        status_key = f'{func_name}_started' if func_name else 'action_started'
        msg = String()
        msg.data = status_key
        self.actionstatus_pub.publish(msg)
        # 有专用模板的动作播 TTS，否则只发状态
        if func_name and self.feedback_dict.get(status_key):
            self._publish_feedback_text(self._format_message(status_key, action=goal_request.action))
        return GoalResponse.ACCEPT

    def _on_bridge_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        prev = self._openclaw_connected
        self._openclaw_connected = data.get('openclaw_connected', False)
        if not prev and self._openclaw_connected:
            self.get_logger().info('OpenClaw 已连接，开始接受指令')

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        with self._state_lock:
            self.interrupt_flag = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """单步执行回调：每次只执行一个动作字符串。"""
        action_str = goal_handle.request.action
        self.interrupt_flag = False
        self.get_logger().info(f'收到动作: {action_str}')
        func_name = action_str  # 兜底，防止 except 块中未赋值

        try:
            all_ok, missing = self._check_all_sensors()
            if not all_ok:
                critical_missing = [
                    cfg.get('label', k) for k, cfg in self._sensors.items()
                    if cfg.get('critical', True) and cfg.get('label', k) in missing
                ]
                if critical_missing:
                    self._publish_status('sensor_warning', names='、'.join(critical_missing))

            match = re.match(r'(\w+)\((.*)\)', action_str.strip())
            if not match:
                self.get_logger().error(f'无法解析动作字符串: {action_str}')
                return self._send_result(goal_handle, success=False, status='action_parse_error',
                                         message=self._format_message('action_parse_error'))

            func_name, args_str = match.groups()
            args = [a.strip() for a in args_str.split(',')] if args_str.strip() else []

            typed_args = []
            for a in args:
                try:
                    typed_args.append(float(a))
                except ValueError:
                    typed_args.append(a)

            self.get_logger().info(f'执行: {func_name}({typed_args})')

            if not hasattr(self, func_name):
                result_success = False
                result_status = 'action_not_found'
                self.get_logger().error(f'未知动作: {func_name}')
            else:
                method = getattr(self, func_name)
                method(*typed_args)
                result_success = getattr(self, '_last_success', True)
                result_status = getattr(self, '_last_status', 'action_executed')
                result_image_path = getattr(self, '_last_image_path', '')
                result_message = getattr(self, '_last_message', '')
        except Exception as e:
            self.get_logger().error(f'执行 {func_name} 异常: {e}')
            result_success = False
            result_status = 'action_error'
            result_message = self._format_message('action_error', error=str(e))

        feedback_msg = StepAction.Feedback()
        feedback_msg.progress = result_status
        goal_handle.publish_feedback(feedback_msg)

        try:
            all_ok, missing = self._check_all_sensors()
            if not all_ok:
                critical_missing = [
                    cfg.get('label', k) for k, cfg in self._sensors.items()
                    if cfg.get('critical', True) and cfg.get('label', k) in missing
                ]
                if critical_missing and result_success:
                    result_message += f'（警告：{",".join(critical_missing)}无数据）'
        except Exception:
            pass

        result = StepAction.Result()
        result.success = result_success
        result.status = result_status
        result.message = result_message
        result.image_path = result_image_path
        self.action_running = False
        goal_handle.succeed()
        return result

    # ======================== 结果 & 状态发布工具 ========================

    def _send_result(self, goal_handle, success, status, image_path='', message=''):
        """辅助方法：直接发送结果（用于解析失败等特殊情况）"""
        feedback_msg = StepAction.Feedback()
        feedback_msg.progress = status
        goal_handle.publish_feedback(feedback_msg)
        self._publish_status(status)
        result = StepAction.Result()
        result.success = success
        result.status = status
        result.message = message or self._format_message(status)
        result.image_path = image_path
        goal_handle.succeed()
        self.action_running = False
        return result

    def _format_message(self, key, **kwargs):
        """从 feedback_dict 查找消息模板并格式化。"""
        template = self.feedback_dict.get(key)
        if template is None:
            return key
        if not kwargs:
            return template
        try:
            return template.format(**kwargs)
        except KeyError as e:
            self.get_logger().warn(f'消息模板格式化失败: key={key}, 缺少占位符 {e}')
            return template

    def _publish_status(self, status_key, silent=False, **kwargs):
        """发布状态码到 actionstatus，silent=True 时不发布 TTS 播报"""
        msg = String()
        msg.data = status_key
        self.actionstatus_pub.publish(msg)
        if not silent:
            message = self._format_message(status_key, **kwargs)
            self._publish_feedback_text(message)
            self.get_logger().info(f'状态发布: {status_key}')

    def _publish_feedback_text(self, text):
        """L1 自动 TTS：委托给 SpeakMixin 的统一队列。"""
        self._enqueue_speak(text)

    def _set_last_result(self, success, status, image_path='', silent=False, **kwargs):
        """由各执行方法调用，设置本次执行结果。silent=True 时不发布 TTS 反馈。"""
        self._last_success = success
        self._last_status = status
        self._last_image_path = image_path
        self._last_message = self._format_message(status, **kwargs)
        if silent:
            msg = String()
            msg.data = status
            self.actionstatus_pub.publish(msg)
        else:
            self._publish_status(status, **kwargs)


# ======================== 主函数 ========================

def main():
    rclpy.init(args=None)
    node = StepActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
