#!/usr/bin/env python3
"""建图导航 Mixin：SLAM、导航、位置记录"""

import os
import string
import time
import subprocess
import tempfile
import shutil

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from concurrent.futures import Future


class NavigationMixin:
    """建图与导航方法，混入 StepActionServer"""

    def _reset_ekf(self):
        """调 /set_pose 归零 EKF 位姿和方位"""
        try:
            from robot_localization.srv import SetPose
        except ImportError:
            self.get_logger().warn('robot_localization.srv 不可用，跳过 EKF 重置')
            return

        client = self.create_client(SetPose, '/set_pose')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_pose 服务不可用，跳过 EKF 重置')
            return
        req = SetPose.Request()
        req.pose.header.frame_id = 'odom_combined'
        req.pose.pose.pose.position.x = 0.0
        req.pose.pose.pose.position.y = 0.0
        req.pose.pose.pose.position.z = 0.0
        req.pose.pose.pose.orientation.x = 0.0
        req.pose.pose.pose.orientation.y = 0.0
        req.pose.pose.pose.orientation.z = 0.0
        req.pose.pose.pose.orientation.w = 1.0
        client.call_async(req)
        self.get_logger().info('EKF 已归零: /set_pose')

    def load_target_points(self):
        """加载 map_mapping.yaml → self.navpose_dict"""
        with open(self.map_mapping_config, 'r') as f:
            target_points = yaml.safe_load(f)
        self.navpose_dict = {}
        self._navpose_name_map = {}     # 中文名 → 字母
        self._symbol_name_map = {}      # 字母 → 中文名（TTS 播报用）
        self._location_descriptions = {}  # 字母 → 描述
        if not target_points:
            return
        for symbol, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = data['position']['x']
            pose.pose.position.y = data['position']['y']
            pose.pose.position.z = data['position']['z']
            pose.pose.orientation.x = data['orientation']['x']
            pose.pose.orientation.y = data['orientation']['y']
            pose.pose.orientation.z = data['orientation']['z']
            pose.pose.orientation.w = data['orientation']['w']
            self.navpose_dict[symbol] = pose
            chinese_name = data.get('name', '').strip('"\'')
            if chinese_name:
                self._navpose_name_map[chinese_name] = symbol
                self._symbol_name_map[symbol] = chinese_name
            desc = data.get('description', '').strip('"\'')
            if desc:
                self._location_descriptions[symbol] = desc

    def get_current_pose(self, name='', description=''):
        """获取当前在 map 坐标系下的位姿，并写入 map_mapping.yaml。"""
        name = name.strip('"\'')
        description = description.strip('"\'')
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup 失败: {e}')
            self._set_last_result(False, 'get_current_pose_failed', name=name)
            return

        if os.path.isfile(self.map_mapping_config):
            with open(self.map_mapping_config, 'r', encoding='utf-8') as f:
                target_points = yaml.safe_load(f) or {}
        else:
            target_points = {}
            os.makedirs(os.path.dirname(self.map_mapping_config), exist_ok=True)

        target_points = {
            k: v for k, v in target_points.items()
            if v.get('name', '').strip('"\'') != name
        }

        used = {
            k for k in target_points.keys()
            if len(k) == 1 and k in string.ascii_uppercase
        }
        next_key = next((ch for ch in string.ascii_uppercase if ch not in used), None)
        if next_key is None:
            self.get_logger().error('位置数量超限（A-Z 已用完）!')
            self._set_last_result(False, 'get_current_pose_failed', name=name)
            return

        if not name:
            name = f'未命名{len(target_points)}'

        entry = {
            'name': name,
            'position': {
                'x': float(transform.transform.translation.x),
                'y': float(transform.transform.translation.y),
                'z': 0.0,
            },
            'orientation': {
                'x': float(transform.transform.rotation.x),
                'y': float(transform.transform.rotation.y),
                'z': float(transform.transform.rotation.z),
                'w': float(transform.transform.rotation.w),
            },
        }
        if description:
            entry['description'] = description
        target_points[next_key] = entry

        try:
            with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8',
                dir=os.path.dirname(self.map_mapping_config),
                delete=False,
            ) as tmp:
                yaml.dump(target_points, tmp, allow_unicode=True,
                          sort_keys=False, default_flow_style=False)
                tmp.flush()
            shutil.move(tmp.name, self.map_mapping_config)
        except OSError as e:
            self.get_logger().error(f'写入 map_mapping.yaml 失败: {e}')
            self._set_last_result(False, 'get_current_pose_failed', name=name)
            return

        self.get_logger().info(
            f'已记录位置 {next_key}: "{name}" -> '
            f'x={target_points[next_key]["position"]["x"]:.2f}, '
            f'y={target_points[next_key]["position"]["y"]:.2f}'
        )
        self._set_last_result(True, 'get_current_pose_success', name=name)

    def set_location_description(self, symbol, description):
        """设置指定位置的描述信息（更新 map_mapping.yaml）。"""
        symbol = symbol.strip('"\'')
        description = description.strip('"\'')
        if os.path.isfile(self.map_mapping_config):
            with open(self.map_mapping_config, 'r', encoding='utf-8') as f:
                target_points = yaml.safe_load(f) or {}
        else:
            target_points = {}

        if not isinstance(target_points.get(symbol), dict):
            self._set_last_result(False, 'set_description_failed', symbol=symbol)
            return

        target_points[symbol]['description'] = description
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8',
                dir=os.path.dirname(self.map_mapping_config),
                delete=False,
            ) as tmp:
                yaml.dump(target_points, tmp, allow_unicode=True,
                          sort_keys=False, default_flow_style=False)
                tmp.flush()
            shutil.move(tmp.name, self.map_mapping_config)
        except OSError as e:
            self.get_logger().error(f'写入描述失败: {e}')
            self._set_last_result(False, 'set_description_failed', symbol=symbol)
            return

        self._location_descriptions[symbol] = description
        self.get_logger().info(f'已更新 {symbol} 的描述: {description}')
        self._set_last_result(True, 'set_description_done', symbol=symbol)

    def navigation(self, point_name):
        """
        导航到指定点。
        point_name: 地图符号（A/B/C...）或中文名称
        """
        point_name = point_name.strip('"\'')
        display_name = point_name  # TTS 播报用的名称（字母键会转为中文名）

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup 失败: {e}')
            self._set_last_result(False, 'navigation_failed', point_name=display_name)
            return

        self.load_target_points()
        self.navigation_finish_flag = False
        self.goal_handle = None
        self._nav_result_status = 'navigation_failed'

        target_pose = self.navpose_dict.get(point_name)
        if target_pose:
            # point_name 是字母键，查中文名用于播报
            display_name = self._symbol_name_map.get(point_name, point_name)
            target_symbol = point_name
        else:
            target_symbol = self._navpose_name_map.get(point_name)
            if target_symbol:
                target_pose = self.navpose_dict.get(target_symbol)

        if not target_pose:
            self.get_logger().error(f'导航目标点 "{point_name}" 不存在')
            self._set_last_result(False, 'navigation_failed', point_name=display_name)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_goal_future = self.navclient.send_goal_async(goal_msg)

        def goal_response_callback(future):
            gh = future.result()
            if not gh or not gh.accepted:
                self.get_logger().error('导航目标被拒绝!')
                self._publish_status('navigation_rejected', point_name=display_name)
                self._nav_result_status = 'navigation_rejected'
                self.navigation_finish_flag = True
                return
            self.goal_handle = gh
            if self.interrupt_flag:
                gh.cancel_goal_async()
                self._nav_result_status = 'navigation_cancelled'
                self.navigation_finish_flag = True
                return
            get_result_future = gh.get_result_async()

            def result_callback(future_result):
                self.navigation_finish_flag = True
                if self.nav_status:
                    self.nav_status = False
                    self._publish_status('navigation_cancelled', point_name=display_name)
                    self._nav_result_status = 'navigation_cancelled'
                elif future_result.result().status == 5:
                    self._publish_status('navigation_cancelled', point_name=display_name)
                    self._nav_result_status = 'navigation_cancelled'
                elif future_result.result().status == 4:
                    self._publish_status('navigation_done', point_name=display_name)
                    desc = self._location_descriptions.get(target_symbol, '')
                    if desc:
                        self._publish_feedback_text(desc)
                    self._nav_result_status = 'navigation_done'
                else:
                    self.get_logger().info(
                        f'导航失败，状态码: {future_result.result().status}'
                    )
                    self._publish_status('navigation_failed', point_name=display_name)
                    self._nav_result_status = 'navigation_failed'

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                break
            time.sleep(0.1)

        self.stop()
        self._set_last_result(True, self._nav_result_status, point_name=display_name, silent=True)

    def slam_start(self):
        """启动 SLAM 建图（非阻塞，由 slam_stop 停止）"""
        self.navigation_stop(silent=True)
        subprocess.run(['ros2', 'service', 'call', '/reset_odometry',
                        'std_srvs/srv/Empty'], check=False)
        self._reset_ekf()
        self._slam_process = subprocess.Popen([
            'ros2', 'run', 'slam_gmapping', 'slam_gmapping',
            '--ros-args', '-p', 'use_sim_time:=false',
        ])
        self.slam_future = Future()
        self._set_last_result(True, 'slam_start')

    def slam_stop(self, silent=False):
        """停止 SLAM 并保存地图。silent=True 时不播 TTS（如被 navigation_start 内部调用）。"""
        if hasattr(self, '_slam_process') and self._slam_process.poll() is None:
            subprocess.Popen([
                'ros2', 'launch', 'wheeltec_nav2', 'save_map.launch.py',
            ])
            time.sleep(5)
            self._kill_process_tree(self._slam_process.pid)
        if not self.slam_future.done():
            self.slam_future.set_result(True)
        self._set_last_result(True, 'slam_stop', silent=silent)

    def navigation_start(self, timeout=60):
        """启动导航功能，阻塞等待 Nav2 就绪（支持中断）"""
        self.slam_stop(silent=True)
        self.navigation_future = Future()
        self._nav_process = subprocess.Popen([
            'ros2', 'launch', 'wheeltec_nav2', 'wheeltec_nav2_model.launch.py',
        ])
        self._publish_feedback_text('导航服务启动中，请稍候…')
        self.get_logger().info('等待 Nav2 就绪...')
        deadline = time.time() + timeout
        ready = False
        while time.time() < deadline:
            if self.interrupt_flag:
                self.get_logger().info('Nav2 等待被中断')
                break
            if self.navclient.wait_for_server(timeout_sec=1.0):
                ready = True
                break
        if ready:
            self.get_logger().info('Nav2 已就绪')
            self._set_last_result(True, 'navigation_start_done')
        else:
            self.get_logger().error(f'Nav2 启动超时（{timeout}s）')
            self._set_last_result(False, 'navigation_start_timeout')

    def navigation_stop(self, silent=False):
        """停止导航功能，silent=True 时不播报"""
        if hasattr(self, '_nav_process') and self._nav_process.poll() is None:
            self._kill_process_tree(self._nav_process.pid)
        if not self.navigation_future.done():
            self.navigation_future.set_result(True)
        self.stop()
        self._set_last_result(True, 'navigation_stop_done', silent=silent)
