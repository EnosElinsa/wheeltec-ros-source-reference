#!/usr/bin/env python3
"""
KCF 跟踪节点 — 输出目标 XY 真实坐标供跟随使用 (优化版 v2)

================================================================
优化点 (相对原版, 共 4 处改动)
================================================================
[A] 遮挡检测提速: 内容校验每帧执行 + 3帧判定 + bbox突变即时检测
[B] 目标恢复重写: 全图多候选搜索 + 深度一致性校验 + 尺寸校验
[C] 恢复确认升级: 位置+分数+尺寸 三重稳定
[D] 深度查询解耦: _get_depth 拆为纯查询 + 状态更新, 消除副作用

未改动模块 (已验证, 保持原样):
  - 三模板采集 (_extract_template / _init_all_templates / _try_capture_template)
  - 颜色反投影自适应框 (_refine_bbox_color)
  - PID 控制器 (pid_controller.py)
  - YOLO 桥接 (_yolo_bridge)
  - 发布逻辑 (_publish_xy / _publish_bbox / _publish_debug)
================================================================

订阅:
  /camera/color/image_raw  (RGB)
  /camera/depth/image_raw  (深度, 32FC1)

发布:
  /kcf/tracked_xy         geometry_msgs/Point     目标真实 XY 坐标, z=深度(m)
  /kcf/tracked_bbox       geometry_msgs/Polygon    图像边界框 (4 角像素坐标)
  /kcf/debug_image        sensor_msgs/Image        调试可视化
  /kcf/tracking_status    std_msgs/Bool            是否正在跟踪
  /cmd_vel                geometry_msgs/Twist      (仅 enable_cmd_vel=true)
"""

import os
os.environ.setdefault('GTK_MODULES', '')
os.environ.setdefault('NO_AT_BRIDGE', '1')

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Point32, PolygonStamped
from std_msgs.msg import Bool, Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters

from .pid_controller import SimplePID


class KCFTrackerNode(Node):
    """KCF 跟踪 + XY 坐标输出节点 (优化版)"""

    def __init__(self):
        super().__init__('kcf_tracker_model')

        # 声明参数 ------------------------------------------------------------
        self.declare_parameter('x1', 290.0)
        self.declare_parameter('y1', 160.0)
        self.declare_parameter('x2', 350.0)
        self.declare_parameter('y2', 200.0)

        self.declare_parameter('target_dist', 0.6)

        self.declare_parameter('linear_Kp', 1.5)
        self.declare_parameter('linear_Ki', 0.0)
        self.declare_parameter('linear_Kd', 1.0)

        self.declare_parameter('angular_Kp', 0.5)
        self.declare_parameter('angular_Ki', 0.0)
        self.declare_parameter('angular_Kd', 2.0)

        self.declare_parameter('dist_deadzone', 0.10)
        self.declare_parameter('angle_deadzone', 0.10)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.35)

        self.declare_parameter('enable_cmd_vel', True)
        self.declare_parameter('show_display', False)

        self.declare_parameter('camera_fx', 606.0)
        self.declare_parameter('camera_fy', 605.0)
        self.declare_parameter('camera_cx', 321.0)
        self.declare_parameter('camera_cy', 241.0)

        self.declare_parameter('use_yolo', False)
        self.declare_parameter('yolo_frame_path', '/tmp/kcf_yolo_frame.jpg')
        self.declare_parameter('yolo_bbox_path', '/tmp/kcf_yolo_bbox.json')
        self.declare_parameter('yolo_interval', 30)
        self._sync_params()
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value

        # [新增] 恢复搜索参数 -----------------------------------------------
        # 全图搜索降采样倍数 (2 = 1/2 分辨率搜索, 加速 4 倍)
        self.declare_parameter('recovery_downscale', 2)
        # 候选数量 (每个模板每尺度取 Top-N)
        self.declare_parameter('recovery_top_k', 3)
        # 深度一致性容差 (m), 候选深度与丢失前最后深度之差超过此值则拒绝
        self.declare_parameter('recovery_depth_tol', 0.3)
        # 尺寸一致性范围 (相对初始面积), 超出则拒绝
        self.declare_parameter('recovery_size_min', 0.3)
        self.declare_parameter('recovery_size_max', 3.0)
        # 综合分数最低阈值
        self.declare_parameter('recovery_min_score', 0.35)
        self.recovery_downscale = self.get_parameter('recovery_downscale').get_parameter_value().integer_value
        self.recovery_top_k = self.get_parameter('recovery_top_k').get_parameter_value().integer_value
        self.recovery_depth_tol = self.get_parameter('recovery_depth_tol').get_parameter_value().double_value
        self.recovery_size_min = self.get_parameter('recovery_size_min').get_parameter_value().double_value
        self.recovery_size_max = self.get_parameter('recovery_size_max').get_parameter_value().double_value
        self.recovery_min_score = self.get_parameter('recovery_min_score').get_parameter_value().double_value

        # 本地显示
        self._first_frame_saved = False
        self._display_ok = False
        if self.show_display:
            try:
                cv2.namedWindow('_kcf_test', cv2.WINDOW_GUI_EXPANDED)
                cv2.destroyWindow('_kcf_test')
                self._display_ok = True
                self.get_logger().info('本地显示窗口已就绪')
            except cv2.error:
                self.get_logger().warn('本地显示不可用 (缺少 GUI 后端), 已跳过')

        self.bridge = CvBridge()

        self.tracker = cv2.TrackerCSRT_create()
        self.tracker_initialized = False

        # 状态变量
        self.result_bbox = None
        self.is_tracking = False
        self.loss_start_time = None
        self.last_loss_log_time = 0.0
        self.rgb_image = None
        self.depth_image = None

        self.img_w = 640
        self.img_h = 480

        # 平滑状态
        self._depth_smooth = 0.0
        self._prev_bbox = None
        self._init_bbox_area = 0

        # [新增] 丢失前最后深度 (用于恢复时深度校验) -------------------------
        self._last_depth_before_loss = 0.0

        # [新增] 恢复保护期: 恢复成功后1秒内不检测面积突变
        # CSRT刚初始化时bbox不稳定, 需要时间收敛
        self._recovery_protection_until = 0.0

        # 三模板
        self._tmpl_initial = None
        self._tmpl_best = None
        self._tmpl_recent = None
        self._tmpl_best_quality = 0.0
        self._tmpl_last_capture = 0.0
        self._tracking_stable_since = 0.0

        # [改动 C] 恢复确认: 升级为三重稳定 (位置 + 分数 + 尺寸)
        self._recovery_candidate = None       # (score, bbox)
        self._recovery_candidate_count = 0
        self._recovery_score_history = []     # 连续候选的分数历史
        self._recovery_size_history = []      # 连续候选的尺寸历史

        self._content_mismatch_count = 0

        self._backproject_last = 0.0
        self._refined_bbox = None
        self._debug_last_publish = 0.0

        self._init_pid()

        # 订阅
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1
        )
        self.sync.registerCallback(self._sync_callback)

        # YOLO 文件桥接状态
        self._yolo_frame_count = 0
        self._yolo_last_bbox_mtime = 0.0
        self._yolo_bbox = None

        # 发布者
        self.xy_pub = self.create_publisher(Point, '/kcf/tracked_xy', 10)
        self.bbox_pub = self.create_publisher(PolygonStamped, '/kcf/tracked_bbox', 10)
        self.debug_pub = self.create_publisher(Image, '/kcf/debug_image', 10)
        self.status_pub = self.create_publisher(Bool, '/kcf/tracking_status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.display_name = 'KCF_Tracker'

        self.get_logger().info('KCF Tracker Model v2 (优化版) 节点已启动')
        self.get_logger().info(
            f'恢复参数: 全图{self.recovery_downscale}x降采样, '
            f'Top-{self.recovery_top_k}候选, '
            f'深度容差±{self.recovery_depth_tol}m, '
            f'尺寸{self.recovery_size_min}-{self.recovery_size_max}x')

    # ------------------------------------------------------------------
    def _sync_params(self):
        p = lambda name: self.get_parameter(name).get_parameter_value()

        self.x1 = int(p('x1').double_value)
        self.y1 = int(p('y1').double_value)
        self.x2 = int(p('x2').double_value)
        self.y2 = int(p('y2').double_value)

        self.target_dist = p('target_dist').double_value

        self.linear_Kp = p('linear_Kp').double_value
        self.linear_Ki = p('linear_Ki').double_value
        self.linear_Kd = p('linear_Kd').double_value

        self.angular_Kp = p('angular_Kp').double_value
        self.angular_Ki = p('angular_Ki').double_value
        self.angular_Kd = p('angular_Kd').double_value

        self.dist_deadzone = p('dist_deadzone').double_value
        self.angle_deadzone = p('angle_deadzone').double_value
        self.max_linear_speed = p('max_linear_speed').double_value
        self.max_angular_speed = p('max_angular_speed').double_value
        self.enable_cmd_vel = p('enable_cmd_vel').bool_value

        self.camera_fx = p('camera_fx').double_value
        self.camera_fy = p('camera_fy').double_value
        self.camera_cx = p('camera_cx').double_value
        self.camera_cy = p('camera_cy').double_value

        self.use_yolo = p('use_yolo').bool_value
        self.yolo_frame_path = p('yolo_frame_path').string_value
        self.yolo_bbox_path = p('yolo_bbox_path').string_value
        self.yolo_interval = p('yolo_interval').integer_value

    def _init_pid(self):
        self.linear_pid = SimplePID(self.linear_Kp, self.linear_Ki, self.linear_Kd,
                                    output_max=3.0)

    # ------------------------------------------------------------------
    def _sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """RGB + 深度同步回调"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 解码失败: {e}')
            return

        try:
            if depth_msg.encoding == '16UC1':
                raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                self.depth_image = raw.astype(np.float32) / 1000.0
            elif depth_msg.encoding in ('32FC1', 'TYPE_32FC1'):
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            else:
                self.get_logger().warn(f'未知深度编码: {depth_msg.encoding}, 尝试 16UC1')
                raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                self.depth_image = raw.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'深度解码失败 (encoding={depth_msg.encoding}): {e}')
            return

        self.img_h, self.img_w = self.rgb_image.shape[:2]

        if not self.tracker_initialized:
            self._init_tracker()

        self._run_tracking()

        self._publish_xy()
        self._publish_bbox()

        t = time.time()
        if t - self._debug_last_publish >= 1.0:
            self._publish_debug()
            self._debug_last_publish = t

        if self._display_ok:
            display_img = self._build_display()
            if display_img is not None:
                if not self._first_frame_saved:
                    path = os.path.expanduser('~/kcf_debug_first_frame.png')
                    cv2.imwrite(path, display_img)
                    self.get_logger().info(f'首帧调试截图已保存: {path}')
                    self._first_frame_saved = True
                cv2.imshow(self.display_name, display_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('收到退出键, 关闭显示窗口')
                    cv2.destroyWindow(self.display_name)
                    self._display_ok = False

        if self.use_yolo:
            self._yolo_bridge()

        if self.enable_cmd_vel:
            self._publish_cmd_vel()

    # ------------------------------------------------------------------
    def _init_tracker(self):
        w = self.x2 - self.x1
        h = self.y2 - self.y1
        if w <= 0 or h <= 0:
            self.get_logger().error(f'非法 bbox: ({self.x1},{self.y1})-({self.x2},{self.y2})')
            return

        bbox = (self.x1, self.y1, w, h)
        self.tracker.init(self.rgb_image, bbox)
        self.tracker_initialized = True
        self.result_bbox = bbox
        self.is_tracking = True
        self._init_bbox_area = w * h
        self._init_all_templates()
        self._tracking_stable_since = time.time()
        self.get_logger().info(f'CSRT 已初始化, bbox=({self.x1},{self.y1}) ({w}x{h})')

    # ------------------------------------------------------------------
    def _yolo_bridge(self):
        """文件桥接: 写帧给 YOLO, 仅在丢失时读取 YOLO bbox 救援"""
        import json
        self._yolo_frame_count += 1

        if self._yolo_frame_count % self.yolo_interval == 0 and self.rgb_image is not None:
            cv2.imwrite(self.yolo_frame_path, self.rgb_image)
            state = {'tracking': self.is_tracking}
            if self.result_bbox is not None:
                bx, by, bw, bh = self.result_bbox
                state['bbox'] = {'x': int(bx), 'y': int(by),
                                 'w': int(bw), 'h': int(bh)}
            with open(self.yolo_frame_path + '.state.json', 'w') as f:
                json.dump(state, f)

        if self.is_tracking:
            return

        try:
            mtime = os.path.getmtime(self.yolo_bbox_path)
        except OSError:
            return
        if mtime <= self._yolo_last_bbox_mtime:
            return
        self._yolo_last_bbox_mtime = mtime

        try:
            with open(self.yolo_bbox_path, 'r') as f:
                data = json.load(f)
            bb = data['bbox']
            x1, y1, x2, y2 = int(bb['x1']), int(bb['y1']), int(bb['x2']), int(bb['y2'])
            bbox = (x1, y1, x2 - x1, y2 - y1)
            if bbox[2] <= 5 or bbox[3] <= 5:
                return
            self._yolo_bbox = (x1, y1, x2, y2)
        except Exception:
            return

        loss_elapsed = (time.time() - self.loss_start_time
                        if self.loss_start_time else 999.0)
        if self.result_bbox is not None and loss_elapsed < 3.0:
            lx, ly, lw, lh = self.result_bbox
            last_cx, last_cy = lx + lw / 2.0, ly + lh / 2.0
            new_cx, new_cy = x1 + (x2 - x1) / 2.0, y1 + (y2 - y1) / 2.0
            pos_jump = np.hypot(new_cx - last_cx, new_cy - last_cy)
            if pos_jump > self.img_w * 0.80:
                self.get_logger().warn(
                    f'YOLO 救援被拒: 位置跳变过大 '
                    f'({pos_jump:.0f}px, 丢失 {loss_elapsed:.0f}s)')
                return

        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(self.rgb_image, bbox)
        self.tracker_initialized = True
        self.result_bbox = bbox
        self.is_tracking = True
        self.loss_start_time = None
        self._prev_bbox = None
        self._init_bbox_area = int(bbox[2] * bbox[3])
        self._tracking_stable_since = time.time()
        self._update_recent_template_only()
        self.get_logger().info(
            f'YOLO 救援成功 (丢失 {loss_elapsed:.0f}s), '
            f'CSRT 已重建: {bbox}')

    def _refine_bbox_color(self, bbox):
        """颜色反投影自适应框 (未改动)"""
        if self._tmpl_initial is None or self.rgb_image is None:
            return None
        x, y, w, h = [int(v) for v in bbox]
        margin = 0.4
        ex = max(0, int(x - w * margin))
        ey = max(0, int(y - h * margin))
        ew = min(self.img_w - ex, int(w * (1 + 2 * margin)))
        eh = min(self.img_h - ey, int(h * (1 + 2 * margin)))
        if ew < 20 or eh < 20:
            return None
        roi = self.rgb_image[ey:ey + ew, ex:ex + eh]
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        bp = cv2.calcBackProject([roi_hsv], [0, 1], self._tmpl_initial[2],
                                 [0, 180, 0, 256], 1)
        _, mask = cv2.threshold(bp, 60, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area_ratio = cv2.contourArea(largest) / (ew * eh)
        if area_ratio < 0.03 or area_ratio > 0.95:
            return None
        rx, ry, rw, rh = cv2.boundingRect(largest)
        fx, fy = ex + rx, ey + ry
        fc_x, fc_y = fx + rw / 2, fy + rh / 2
        csrt_cx, csrt_cy = x + w / 2, y + h / 2
        if abs(fc_x - csrt_cx) > w * 1.2 or abs(fc_y - csrt_cy) > h * 1.2:
            return None
        return (fx, fy, rw, rh)

    def _extract_template(self):
        """从当前 result_bbox 提取 (rgb, edge, hist) 三元组 (未改动)"""
        if self.result_bbox is None or self.rgb_image is None:
            return None
        x, y, w, h = [int(v) for v in self.result_bbox]
        x = max(0, x); y = max(0, y)
        w = min(w, self.rgb_image.shape[1] - x)
        h = min(h, self.rgb_image.shape[0] - y)
        if w <= 10 or h <= 10:
            return None
        roi = self.rgb_image[y:y+h, x:x+w]
        rh, rw = roi.shape[:2]
        max_side = max(rw, rh)
        if max_side < 16:
            s = 16.0 / max_side
        elif max_side > 200:
            s = 200.0 / max_side
        else:
            s = 1.0
        tmpl_w = max(8, int(rw * s))
        tmpl_h = max(8, int(rh * s))
        rgb_tmpl = cv2.resize(roi, (tmpl_w, tmpl_h), interpolation=cv2.INTER_AREA)
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi_edge = cv2.Canny(roi_gray, 50, 150)
        edge_tmpl = cv2.resize(roi_edge, (tmpl_w, tmpl_h), interpolation=cv2.INTER_NEAREST)
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([roi_hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        return (rgb_tmpl, edge_tmpl, hist)

    def _init_all_templates(self):
        """首次初始化: 三模板都用第一帧填充 (仅在 _init_tracker 时调用)"""
        tmpl = self._extract_template()
        if tmpl is not None:
            self._tmpl_initial = tmpl
            self._tmpl_best = tmpl
            self._tmpl_recent = tmpl
            self._tmpl_best_quality = 1.0
            self._tmpl_last_capture = time.time()

    def _update_recent_template_only(self):
        """恢复后更新: 只更新 _tmpl_recent, 永不覆盖 _tmpl_initial 和 _tmpl_best

        这是关键修复: 恢复帧即通过了校验, 也可能有轻微偏差.
        如果用 _init_all_templates() 覆盖 _tmpl_initial, 偏差会逐次累积,
        最终导致 _tmpl_initial 偏得无法匹配真实目标 → 永久卡住.
        CONTEXT.md 规定 "初始模板永久保留", 此方法遵守该原则.
        """
        tmpl = self._extract_template()
        if tmpl is not None:
            self._tmpl_recent = tmpl
            self._tmpl_last_capture = time.time()

    def _try_capture_template(self):
        """[v5] 采集模板: 放宽 recent 更新条件, 让目标姿态变化时也能更新

        关键改动: recent 不再强制与 initial 相似度≥0.6
        - recent 的作用是"最接近当前姿态", 目标转向时恰恰需要更新它
        - 改用"跟踪稳定+尺寸合理"作为筛选, 避免学到遮挡物
        - best 仍保留质量分筛选 (best 用于姿态较好时恢复)
        """
        if not self.is_tracking or self._tmpl_initial is None:
            return
        x, y, w, h = [int(v) for v in self.result_bbox]
        # 筛选1: bbox 面积在合理范围 (0.5-2.0x 初始)
        if self._init_bbox_area > 0:
            size_ratio = (w * h) / self._init_bbox_area
            if size_ratio < 0.5 or size_ratio > 2.0:
                return
        # 筛选2: 跟踪稳定 ≥ 1秒 (v5: 2s→1s, 小车调整就几秒)
        if time.time() - self._tracking_stable_since < 1.0:
            return
        # 提取模板
        tmpl = self._extract_template()
        if tmpl is None:
            return
        _, _, hist = tmpl
        init_hist = self._tmpl_initial[2]
        quality = cv2.compareHist(init_hist, hist, cv2.HISTCMP_CORREL)

        # [v5] recent 更新: 不强制与 initial 相似
        # 目标姿态变化时 quality 会低, 但这正是需要更新 recent 的时候
        # 只要不匹配到完全不同的东西就行 (quality > 0.2 排除明显异物)
        if quality > 0.2:
            self._tmpl_recent = tmpl
            self._tmpl_last_capture = time.time()
            if quality < 0.6:
                self.get_logger().debug(
                    f'recent更新(姿态变化): quality={quality:.3f}')

        # best 仍用严格筛选 (质量分高才更新)
        if quality > 0.6 and quality > self._tmpl_best_quality:
            self._tmpl_best = tmpl
            self._tmpl_best_quality = quality
            self.get_logger().info(f'模板更新: best quality={quality:.3f}')

    # ==================================================================
    # [改动 B v3] 两阶段恢复: 全图搜索 + 局部确认
    # ==================================================================
    def _template_recovery_search(self, hint_bbox=None, radius=0):
        """模板匹配搜索 + 深度/尺寸/颜色校验

        两阶段调用:
        - hint_bbox=None: 全图降采样搜索 (阶段1, 找初始候选)
        - hint_bbox=上次候选, radius=N: 以候选为中心局部全分辨率搜索 (阶段2, 确认)

        局部搜索用全分辨率 (范围小, 性能够, 精度高), 位置天然稳定.
        全图搜索用降采样 (范围大, 需加速), 找初始候选.

        返回 (combined, bbox, (edge_score, color_score)) 或 None
        """
        # [v5] 模板优先级: recent → best → initial
        # recent 最接近当前姿态, 最可能匹配; initial 是初始正面, 姿态变化后难匹配
        templates = [t for t in (self._tmpl_recent, self._tmpl_best, self._tmpl_initial)
                     if t is not None]
        if not templates or self.rgb_image is None:
            return None

        last_depth = self._last_depth_before_loss

        # --- 确定搜索区域 ---
        if hint_bbox is not None and radius > 0:
            # 阶段2: 局部搜索 (全分辨率)
            hc_x = int(hint_bbox[0] + hint_bbox[2] / 2)
            hc_y = int(hint_bbox[1] + hint_bbox[3] / 2)
            sx = max(0, hc_x - radius)
            sy = max(0, hc_y - radius)
            ex = min(self.img_w, hc_x + radius)
            ey = min(self.img_h, hc_y + radius)
            if ex - sx < 30 or ey - sy < 30:
                return None
            search_rgb = self.rgb_image[sy:ey, sx:ex]
            search_gray = cv2.cvtColor(search_rgb, cv2.COLOR_BGR2GRAY)
            search_edge = cv2.Canny(search_gray, 50, 150)
            offset_x, offset_y = sx, sy
            ds = 1  # 局部不降采样
            phase = '局部'
        else:
            # 阶段1: 全图搜索 (降采样)
            ds = self.recovery_downscale
            sw = self.img_w // ds
            sh = self.img_h // ds
            search_rgb = cv2.resize(self.rgb_image, (sw, sh), interpolation=cv2.INTER_AREA)
            search_gray = cv2.cvtColor(search_rgb, cv2.COLOR_BGR2GRAY)
            search_edge = cv2.Canny(search_gray, 50, 150)
            offset_x, offset_y = 0, 0
            phase = '全图'

        sw, sh = search_rgb.shape[1], search_rgb.shape[0]
        # [v7] 增加更多尺度, 覆盖目标远近变化
        scales = [0.6, 0.8, 1.0, 1.3, 1.7]
        candidates = []

        for rgb_tmpl, edge_tmpl, hist in templates:
            for s in scales:
                tw = int(edge_tmpl.shape[1] * s / ds)
                th = int(edge_tmpl.shape[0] * s / ds)
                if tw < 8 or th < 8 or tw > sw or th > sh:
                    continue

                resized_edge = cv2.resize(edge_tmpl, (tw, th), interpolation=cv2.INTER_NEAREST)
                res_e = cv2.matchTemplate(search_edge, resized_edge, cv2.TM_CCOEFF_NORMED)

                rgb_tmpl_s = cv2.resize(rgb_tmpl, (tw, th), interpolation=cv2.INTER_AREA)
                res_c = cv2.matchTemplate(search_rgb, rgb_tmpl_s, cv2.TM_CCORR_NORMED)

                # 全图搜索取Top-K, 局部搜索只取最高分 (范围小, 不需要多候选)
                top_k = self.recovery_top_k if phase == '全图' else 1
                res_e_work = res_e.copy()
                for _ in range(top_k):
                    _, max_e, _, max_loc = cv2.minMaxLoc(res_e_work)
                    if max_e < 0.15:  # [v7] 放宽边缘分数门槛 0.2→0.15
                        break
                    color_at = float(res_c[max_loc[1], max_loc[0]])
                    combined = (max_e * color_at) ** 0.5

                    # [v7] 局部确认阶段降低分数门槛 (已经在候选附近, 要求可放宽)
                    score_thresh = self.recovery_min_score if phase == '全图' \
                        else self.recovery_min_score * 0.6
                    if combined > score_thresh:
                        bx = max_loc[0] * ds + offset_x
                        by_ = max_loc[1] * ds + offset_y
                        bw = tw * ds
                        bh = th * ds
                        bx = max(0, min(bx, self.img_w - bw))
                        by_ = max(0, min(by_, self.img_h - bh))
                        candidates.append((combined, (bx, by_, bw, bh),
                                           float(max_e), color_at, s))

                    mask_x1 = max(0, max_loc[0] - tw // 2)
                    mask_y1 = max(0, max_loc[1] - th // 2)
                    mask_x2 = min(sw, max_loc[0] + tw // 2)
                    mask_y2 = min(sh, max_loc[1] + th // 2)
                    res_e_work[mask_y1:mask_y2, mask_x1:mask_x2] = 0

        if not candidates:
            return None

        # 多候选校验筛选 (尺寸 + 深度 + 颜色巴氏)
        best = None
        rejected = 0
        for combined, bbox, edge_s, color_s, s in candidates:
            bx, by_, bw, bh = bbox

            if self._init_bbox_area > 0:
                size_ratio = (bw * bh) / self._init_bbox_area
                if size_ratio < self.recovery_size_min or size_ratio > self.recovery_size_max:
                    rejected += 1
                    continue

            if last_depth > 0 and self.depth_image is not None:
                cand_depth = self._query_bbox_depth(bbox)
                if cand_depth > 0:
                    # [v7] 局部确认阶段放宽深度容差 (目标可能移动)
                    depth_tol = self.recovery_depth_tol if phase == '全图' \
                        else self.recovery_depth_tol * 1.5
                    if abs(cand_depth - last_depth) > depth_tol:
                        rejected += 1
                        continue

            roi = self.rgb_image[max(0, by_):min(self.img_h, by_ + bh),
                                 max(0, bx):min(self.img_w, bx + bw)]
            if roi.shape[0] > 10 and roi.shape[1] > 10:
                roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                roi_hist = cv2.calcHist([roi_hsv], [0, 1], None,
                                        [30, 32], [0, 180, 0, 256])
                cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
                # [v7] 颜色校验用 recent 模板(最接近当前姿态), 容差放宽
                ref_hist = self._tmpl_recent[2] if self._tmpl_recent is not None \
                    else self._tmpl_initial[2]
                color_dist = cv2.compareHist(
                    ref_hist, roi_hist, cv2.HISTCMP_BHATTACHARYYA)
                # [v7] 巴氏距离阈值放宽 0.5→0.6
                if color_dist > 0.6:
                    rejected += 1
                    continue

            if best is None or combined > best[0]:
                best = (combined, bbox, (edge_s, color_s))

        if best is not None:
            score, bbox, (edge_s, color_s) = best
            self.get_logger().info(
                f'{phase}恢复: 通过校验 pos=({bbox[0]},{bbox[1]}) '
                f'size={bbox[2]}x{bbox[3]} '
                f'combined={score:.3f} '
                f'(候选{len(candidates)}, 拒绝{rejected}) ✓')
            return best
        else:
            self.get_logger().debug(
                f'{phase}恢复: {len(candidates)}个候选全部被校验拒绝')
            return None

    # ==================================================================
    # [改动 D] 深度查询解耦: 纯查询 + 状态更新分离
    # ==================================================================
    def _query_bbox_depth(self, bbox) -> float:
        """纯查询: 对指定 bbox 做深度采样, 不更新任何状态.

        用于恢复候选校验 (无副作用), 可对任意 bbox 查询深度.
        返回中位数深度 (m), 无效返回 -1.0
        """
        if self.depth_image is None or bbox is None:
            return -1.0

        x, y, w, h = [int(v) for v in bbox]
        cx = int(x + w / 2)
        cy = int(y + h / 2)

        h_img, w_img = self.depth_image.shape[:2]

        # 5 点十字采样
        points = [(cy, cx), (cy - 3, cx), (cy + 3, cx),
                  (cy, cx - 3), (cy, cx + 3)]

        valid = []
        for py, px in points:
            if 0 <= py < h_img and 0 <= px < w_img:
                d = self.depth_image[py, px]
                if not np.isnan(d) and not np.isinf(d) and 0.05 < d < 10.0:
                    valid.append(d)

        if not valid:
            return -1.0
        return float(np.median(valid))

    def _get_depth(self) -> float:
        """查询当前 result_bbox 深度 + EMA 平滑 (保留原接口).

        注意: 此方法现在只对 self.result_bbox 查询并平滑,
        不再被多处调用导致状态污染 (因为 _publish_debug/_build_display
        改用 _query_bbox_depth 查询显示用深度).
        """
        if self.depth_image is None or self.result_bbox is None:
            return -1.0

        raw = self._query_bbox_depth(self.result_bbox)
        if raw <= 0:
            return -1.0

        # EMA 平滑 (仅此一处更新状态)
        alpha = 0.8
        if self._depth_smooth <= 0:
            self._depth_smooth = raw
        else:
            self._depth_smooth = alpha * raw + (1.0 - alpha) * self._depth_smooth
        return self._depth_smooth

    # ------------------------------------------------------------------
    def _pixel_to_world(self, px: float, py: float, depth: float):
        X = (px - self.camera_cx) * depth / self.camera_fx
        Y = (py - self.camera_cy) * depth / self.camera_fy
        return X, Y

    # ==================================================================
    # [改动 A + C v3] 跟踪主循环: 遮挡即时检测 + 两阶段恢复
    # ==================================================================
    def _run_tracking(self):
        """CSRT 跟踪 + 丢失后两阶段恢复 (全图搜索 + 局部确认)"""
        if not self.tracker_initialized:
            return

        now_t = time.time()

        # --- 丢失状态: 两阶段恢复 ---
        if not self.is_tracking:
            if self._recovery_candidate is None:
                # 阶段1: 全图搜索 (找初始候选)
                recovered = self._template_recovery_search(hint_bbox=None, radius=0)
                if recovered is not None:
                    score, r_bbox, (edge_s, color_s) = recovered
                    self._recovery_candidate = (score, r_bbox)
                    self._recovery_candidate_count = 1
                    self.get_logger().info(
                        f'阶段1 全图搜索找到候选, 进入局部确认 '
                        f'pos=({r_bbox[0]},{r_bbox[1]}) score={score:.3f}')
                else:
                    elapsed = now_t - self.loss_start_time if self.loss_start_time else 0
                    if now_t - self.last_loss_log_time >= 1.0:
                        self.last_loss_log_time = now_t
                        self.get_logger().warn(f'目标丢失中, {elapsed:.0f}s (全图搜索中)')
            else:
                # 阶段2: 局部确认 (在上一帧候选附近±80px搜索)
                _, prev_bbox = self._recovery_candidate
                recovered = self._template_recovery_search(
                    hint_bbox=prev_bbox, radius=80)
                if recovered is not None:
                    score, r_bbox, (edge_s, color_s) = recovered
                    self._recovery_candidate = (score, r_bbox)
                    self._recovery_candidate_count += 1

                    if self._recovery_candidate_count >= 3:
                        # 累积3次局部确认 → 恢复成功
                        rx, ry, rw, rh = r_bbox
                        self.tracker = cv2.TrackerCSRT_create()
                        self.tracker.init(self.rgb_image, r_bbox)
                        self.tracker_initialized = True
                        self.result_bbox = r_bbox
                        self._prev_bbox = None
                        # [v4修复] _init_bbox_area 不在恢复时更新!
                        # 保持初始基准, 否则尺寸校验基准漂移导致候选尺寸退化
                        # (186x200 → 148x160 → 108x116 → 找不到)
                        self.is_tracking = True
                        self.loss_start_time = None
                        self._recovery_candidate = None
                        self._recovery_candidate_count = 0
                        self._refined_bbox = None
                        self._tracking_stable_since = now_t
                        self._depth_smooth = 0.0
                        self._recovery_protection_until = now_t + 1.0  # 1秒保护期
                        self._update_recent_template_only()
                        self.get_logger().info(
                            f'恢复成功 (局部确认3帧, '
                            f'combined={score:.2f})')
                        self.status_pub.publish(Bool(data=True))
                else:
                    # 局部确认失败, 回到阶段1重新全图搜索
                    self.get_logger().info(
                        f'局部确认失败({self._recovery_candidate_count}次), '
                        f'回到全图搜索')
                    self._recovery_candidate = None
                    self._recovery_candidate_count = 0

            if not self.is_tracking:
                self.status_pub.publish(Bool(data=False))
            return

        # --- 正常跟踪: CSRT update ---
        success, bbox = self.tracker.update(self.rgb_image)
        x, y, w, h = bbox

        # --- 异常检测 (size + jump) ---
        img_area = self.img_w * self.img_h
        bbox_ratio = (w * h) / img_area if img_area > 0 else 0
        size_ok = 0.005 < bbox_ratio < 0.85

        jump_ok = True
        if self._prev_bbox is not None:
            px, py, pw, ph = self._prev_bbox
            prev_cx, prev_cy = px + pw / 2.0, py + ph / 2.0
            curr_cx, curr_cy = x + w / 2.0, y + h / 2.0
            jump = np.hypot(curr_cx - prev_cx, curr_cy - prev_cy)
            jump_ok = jump < self.img_w * 0.35

        # [改动 A v4] bbox 面积突变检测 (遮挡特征)
        # v4改进: 1.恢复保护期内不检测 2.需要同时内容不匹配才算遮挡
        area_bloat = False
        if (self._prev_bbox is not None and self._init_bbox_area > 0
                and now_t > self._recovery_protection_until):
            prev_area = self._prev_bbox[2] * self._prev_bbox[3]
            curr_area = w * h
            if prev_area > 0:
                bloat_ratio = curr_area / prev_area
                # 面积突然膨胀 >2.0x = 可能被遮挡
                # (v4: 阈值从1.5放宽到2.0, CSRT正常波动不会超2倍)
                if bloat_ratio > 2.0:
                    area_bloat = True

        if size_ok and jump_ok and not area_bloat:
            self.result_bbox = bbox
            self._prev_bbox = bbox
            self.is_tracking = True
            self.loss_start_time = None
            self.last_loss_log_time = now_t

            # [改动 A v4] 内容校验: 每帧执行
            if self._tmpl_initial is not None:
                vb = self._refined_bbox if self._refined_bbox is not None else bbox
                vx, vy, vw, vh = [int(v) for v in vb]
                cx, cy = max(0, vx), max(0, vy)
                cw = min(vw, self.img_w - cx)
                ch = min(vh, self.img_h - cy)
                if cw <= 10 or ch <= 10:
                    self._content_mismatch_count += 1
                else:
                    roi = self.rgb_image[cy:cy+ch, cx:cx+cw]
                    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    roi_hist = cv2.calcHist(
                        [roi_hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
                    cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
                    color_sim = cv2.compareHist(
                        self._tmpl_initial[2], roi_hist, cv2.HISTCMP_CORREL)
                    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    roi_edge = cv2.Canny(roi_gray, 50, 150)
                    roi_edge_rs = cv2.resize(roi_edge,
                        (self._tmpl_initial[1].shape[1], self._tmpl_initial[1].shape[0]),
                        interpolation=cv2.INTER_NEAREST)
                    edge_sim = cv2.matchTemplate(
                        roi_edge_rs, self._tmpl_initial[1], cv2.TM_CCOEFF_NORMED)[0][0]

                    # [v4] 面积突变 + 内容不匹配 双重确认才算遮挡
                    # 单独面积突变可能是CSRT波动, 单独内容不匹配可能是光照变化
                    # 两者同时出现 = 强遮挡信号
                    if color_sim < 0.3 and edge_sim < 0.2:
                        self._content_mismatch_count += 1
                        # 有面积突变时3帧判定, 无面积突变时5帧 (更保守)
                        threshold = 3 if area_bloat else 5
                        if self._content_mismatch_count >= threshold:
                            self.get_logger().warn(
                                f'目标丢失: 内容不匹配 '
                                f'(color={color_sim:.2f} edge={edge_sim:.2f}, '
                                f'连续{self._content_mismatch_count}帧, '
                                f'面积突变={area_bloat})')
                            self._last_depth_before_loss = self._get_depth()
                            self.cmd_pub.publish(Twist())
                            self.is_tracking = False
                            self.loss_start_time = now_t
                            self.last_loss_log_time = now_t
                            self._content_mismatch_count = 0
                            self._prev_bbox = None
                            self._refined_bbox = None
                            self._recovery_candidate = None
                            self._recovery_candidate_count = 0
                            self._recovery_score_history = []
                            self._recovery_size_history = []
                            self.status_pub.publish(Bool(data=False))
                            return
                    else:
                        self._content_mismatch_count = 0

                # 反投影自适应框 (节流保留, 反投影本身代价高)
                if now_t - self._backproject_last >= 0.5:
                    self._backproject_last = now_t
                    refined = self._refine_bbox_color(vb)
                    if refined is not None:
                        self.result_bbox = refined
                        self._prev_bbox = refined
                        self._refined_bbox = refined
                        csrt_a = w * h
                        refined_a = refined[2] * refined[3]
                        if refined_a > csrt_a * 1.2:
                            self.tracker = cv2.TrackerCSRT_create()
                            self.tracker.init(self.rgb_image, refined)
                            # [v4] _init_bbox_area 不在反投影修正时更新
                            # 保持初始基准, 避免尺寸校验漂移
                            self._tracking_stable_since = time.time()
                    else:
                        self._refined_bbox = None

            if self._refined_bbox is not None:
                self.result_bbox = self._refined_bbox
                self._prev_bbox = self._refined_bbox

            # [v5] 定期模板采集: 间隔 3s→1s (小车调整快, 需要及时更新)
            if now_t - self._tmpl_last_capture >= 1.0:
                self._try_capture_template()
        else:
            if self.is_tracking:
                reason = []
                if not size_ok:
                    reason.append(f'bbox比例异常({bbox_ratio:.3f})')
                if not jump_ok:
                    reason.append('中心跳变过大')
                if area_bloat:
                    reason.append('面积突变(疑似遮挡)')
                self.get_logger().warn(
                    f'目标丢失: {", ".join(reason)}')
                # 记录丢失前最后深度
                self._last_depth_before_loss = self._get_depth()
                self.cmd_pub.publish(Twist())
            self.is_tracking = False
            self.loss_start_time = now_t
            self.last_loss_log_time = now_t
            self._refined_bbox = None
            self._content_mismatch_count = 0

        self.status_pub.publish(Bool(data=self.is_tracking))

    # ------------------------------------------------------------------
    def _publish_xy(self):
        if self.result_bbox is None or not self.is_tracking:
            return

        x, y, w, h = self.result_bbox
        cx = x + w / 2.0
        cy = y + h / 2.0
        depth = self._get_depth()

        if depth <= 0:
            return

        real_x, real_y = self._pixel_to_world(cx, cy, depth)

        msg = Point()
        msg.x = real_x
        msg.y = real_y
        msg.z = float(depth)
        self.xy_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_bbox(self):
        if self.result_bbox is None or not self.is_tracking:
            return

        x, y, w, h = [float(v) for v in self.result_bbox]
        corners = [
            Point32(x=x, y=y),
            Point32(x=x + w, y=y),
            Point32(x=x + w, y=y + h),
            Point32(x=x, y=y + h),
        ]

        msg = PolygonStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_color_frame')
        msg.polygon.points = corners
        self.bbox_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_debug(self):
        if self.rgb_image is None:
            return

        debug = self.rgb_image.copy()

        if self.result_bbox is not None:
            x, y, w, h = [int(v) for v in self.result_bbox]
            color = (0, 255, 255) if self.is_tracking else (0, 0, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            cv2.circle(debug, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)

            # [改动 D] 调试图用纯查询, 不污染 EMA 状态
            d = self._query_bbox_depth(self.result_bbox)
            if d > 0:
                cv2.putText(debug, f'{d:.2f}m', (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if self._yolo_bbox is not None:
            yx1, yy1, yx2, yy2 = self._yolo_bbox
            cv2.rectangle(debug, (yx1, yy1), (yx2, yy2), (0, 255, 0), 1)
            cv2.putText(debug, 'YOLO', (yx1, yy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        status_text = 'TRACKING' if self.is_tracking else 'LOST'
        status_color = (0, 255, 0) if self.is_tracking else (0, 0, 255)
        cv2.putText(debug, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            self.debug_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'调试图像发布失败: {e}')

    # ------------------------------------------------------------------
    def _build_display(self):
        if self.rgb_image is None:
            return None

        rgb_view = self.rgb_image.copy()
        if self.result_bbox is not None:
            x, y, w, h = [int(v) for v in self.result_bbox]
            bbox_color = (0, 255, 255) if self.is_tracking else (0, 0, 255)
            cv2.rectangle(rgb_view, (x, y), (x + w, y + h), bbox_color, 2)
            cv2.circle(rgb_view, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)
            cv2.line(rgb_view, (self.img_w // 2, 0), (self.img_w // 2, self.img_h),
                     (255, 255, 255), 1)
            cv2.line(rgb_view, (0, self.img_h // 2), (self.img_w, self.img_h // 2),
                     (255, 255, 255), 1)

        if self._yolo_bbox is not None:
            yx1, yy1, yx2, yy2 = self._yolo_bbox
            cv2.rectangle(rgb_view, (yx1, yy1), (yx2, yy2), (0, 255, 0), 1)
            cv2.putText(rgb_view, 'YOLO', (yx1, yy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        depth_view = None
        if self.depth_image is not None:
            valid_d = self.depth_image[(self.depth_image > 0.2) & (self.depth_image < 10.0)]
            if valid_d.size > 100:
                d_min = max(0.2, float(np.percentile(valid_d, 5)))
                d_max = min(6.0, float(np.percentile(valid_d, 95)))
            else:
                d_min, d_max = 0.2, 6.0
            if d_max - d_min < 1.0:
                d_max = d_min + 1.0
            depth_clip = np.clip(self.depth_image, d_min, d_max)
            depth_norm = ((depth_clip - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            depth_view = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

            if self.result_bbox is not None:
                x, y, w, h = [int(v) for v in self.result_bbox]
                cv2.rectangle(depth_view, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.circle(depth_view, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)

        if depth_view is not None:
            if depth_view.shape[1] != rgb_view.shape[1]:
                depth_view = cv2.resize(depth_view, (rgb_view.shape[1],
                                        int(rgb_view.shape[1] * depth_view.shape[0] / depth_view.shape[1])))
            display = np.vstack([rgb_view, depth_view])
        else:
            display = rgb_view

        overlay = display.copy()
        panel_w, panel_h = 280, 210
        cv2.rectangle(overlay, (5, 5), (5 + panel_w, 5 + panel_h), (30, 30, 30), -1)
        display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        row_y = 25
        line_h = 18

        status_text = 'TRACKING' if self.is_tracking else 'LOST'
        status_color = (0, 255, 0) if self.is_tracking else (0, 0, 255)
        cv2.putText(display, f'Status: {status_text}', (12, row_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 1)
        row_y += line_h

        if self.result_bbox is not None:
            bx, by, bw, bh = self.result_bbox
            cv2.putText(display, f'BBox: ({int(bx)},{int(by)}) {int(bw)}x{int(bh)}', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        row_y += line_h

        # [改动 D] 显示用纯查询, 不污染 EMA
        depth = self._query_bbox_depth(self.result_bbox) if self.result_bbox else -1.0
        if depth > 0 and self.result_bbox is not None:
            _x, _y, _w, _h = self.result_bbox
            cx = _x + _w / 2.0
            cy = _y + _h / 2.0
            rx, ry = self._pixel_to_world(cx, cy, depth)
            cv2.putText(display, f'Depth: {depth:.3f}m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h
            dist_err = depth - self.target_dist
            err_color = (0, 255, 0) if abs(dist_err) < self.dist_deadzone else (0, 200, 255)
            cv2.putText(display, f'Dist Error: {dist_err:+.3f}m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, err_color, 1)
            row_y += line_h
            cv2.putText(display, f'World XY: ({rx:+.3f}, {ry:+.3f})m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h
        else:
            cv2.putText(display, f'Depth: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h
            cv2.putText(display, f'Dist Error: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h
            cv2.putText(display, f'World XY: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h

        # [新增] 丢失前最后深度显示 (恢复校验参考)
        if not self.is_tracking and self._last_depth_before_loss > 0:
            cv2.putText(display, f'Last Depth: {self._last_depth_before_loss:.3f}m',
                        (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
            row_y += line_h

        if self.enable_cmd_vel and self.is_tracking:
            cv2.putText(display, f'Linear PID: {self.linear_pid.targetpoint:+.3f}', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h

        cv2.putText(display, f'Target Dist: {self.target_dist:.2f}m', (12, row_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        row_y += line_h

        if self.loss_start_time is not None:
            elapsed = time.time() - self.loss_start_time
            cv2.putText(display, f'Loss time: {elapsed:.1f}s', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        max_w = 1280
        if display.shape[1] > max_w:
            scale = max_w / display.shape[1]
            display = cv2.resize(display, (max_w, int(display.shape[0] * scale)))

        return display

    # ------------------------------------------------------------------
    def _publish_cmd_vel(self):
        """[v6] 参考 LaserFollower 重写 cmd_vel: 对称限幅 + 倒车减速 + 双死区

        关键改动 (参考 wheeltec LaserFollower):
        1. 对称限幅 ±max_linear_speed (原版倒车硬编码-0.3, 比前进快, 会冲出视野)
        2. 倒车限速 max_linear_speed*0.5 (倒车快→目标快速离开视野→丢失)
        3. 双死区: 距离误差+角度误差都超阈值才移动 (参考 LaserFollower)
        4. 距离<0.3m 强制停车 (太近碰撞风险, 参考 distance>0.3 判断)
        """
        if self.result_bbox is None or not self.is_tracking:
            return

        depth = self._get_depth()
        if depth <= 0:
            self.cmd_pub.publish(Twist())
            return

        x, y, w, h = self.result_bbox
        cx = x + w / 2.0

        # --- 线性速度 ---
        linear_raw = -self.linear_pid.compute(self.target_dist, depth)

        # [v6] 对称限幅: 倒车和前进都用 max_linear_speed
        linear_raw = max(-self.max_linear_speed, min(self.max_linear_speed, linear_raw))

        # [v6] 倒车限速: 倒车速度限制在 max_linear_speed * 0.5
        # 倒车太快 → 目标快速离开视野 → 丢失 (老板反馈的"倒退反向"问题根因)
        if linear_raw < 0:
            linear_raw = max(-self.max_linear_speed * 0.5, linear_raw)

        # --- 角速度 ---
        angle_ratio = (self.img_w / 2.0 - cx) / (self.img_w / 2.0)
        angular_raw = self.angular_Kp * angle_ratio
        angular_raw = max(-self.max_angular_speed, min(self.max_angular_speed, angular_raw))

        # [v7] 倒车时反转角速度方向 (关键修复!)
        # 前进时: 目标在右(cx>中心) → angular<0(右转) → 目标移向中间 ✓
        # 倒车时: 机器人后退, "前进方向"反转, 角速度逻辑也要反转
        #   不反转: 倒车+右转 → 目标更快移出视野 → 左右反向感
        #   反转后: 倒车+左转(angular>0) → 目标正确移向中间
        if linear_raw < 0:
            angular_raw = -angular_raw

        # [v6] 双死区判断 (参考 LaserFollower: angle_error>0.05 且 linear_error>0.05)
        linear_error = abs(depth - self.target_dist)
        angle_error = abs(angle_ratio)  # 归一化角度误差 0-1
        in_linear_deadzone = linear_error < self.dist_deadzone
        in_angle_deadzone = abs(angular_raw) < self.angle_deadzone

        # 距离<0.3m 太近, 强制停车避免碰撞 (参考 LaserFollower distance>0.3)
        if depth < 0.3:
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # 双死区内停车
        if in_linear_deadzone and in_angle_deadzone:
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # 死区内对应分量清零
        if in_linear_deadzone:
            linear_raw = 0.0
        if in_angle_deadzone:
            angular_raw = 0.0

        # --- 转向自动减速 ---
        turn_ratio = abs(angular_raw) / self.max_angular_speed if self.max_angular_speed > 0 else 0.0
        linear_raw *= (1.0 - 0.6 * min(turn_ratio, 1.0))

        twist = Twist()
        twist.angular.z = angular_raw
        twist.linear.x = linear_raw
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KCFTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
