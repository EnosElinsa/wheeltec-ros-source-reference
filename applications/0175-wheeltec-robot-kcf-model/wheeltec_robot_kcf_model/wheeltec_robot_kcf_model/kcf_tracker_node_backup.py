#!/usr/bin/env python3
"""
KCF 跟踪节点 — 输出目标 XY 真实坐标供跟随使用。

订阅:
  /camera/color/image_raw  (RGB)
  /camera/depth/image_raw  (深度, 32FC1)

发布:
  /kcf/tracked_xy         geometry_msgs/Point     目标真实 XY 坐标, z=深度(m)
  /kcf/tracked_bbox       geometry_msgs/Polygon    图像边界框 (4 角像素坐标)
  /kcf/debug_image        sensor_msgs/Image        调试可视化
  /kcf/tracking_status    std_msgs/Bool            是否正在跟踪
  /cmd_vel                geometry_msgs/Twist      (仅 enable_cmd_vel=true)

参数:
  x1, y1, x2, y2          初始 bbox (px), 默认 290,160,350,200
  target_dist              目标跟随距离 (m), 默认 0.6
  linear_Kp/Ki/Kd          线性 PID, 默认 3.0/0.0/1.0
  angular_Kp/Ki/Kd         角速度 PID, 默认 0.5/0.0/2.0
  dist_deadzone            距离死区 (mm), 默认 30
  angle_deadzone           角度死区 (rad/s), 默认 0.05
  max_linear_speed         最大线速度 (m/s), 默认 0.5
  max_angular_speed        最大角速度 (rad/s), 默认 0.5
  enable_cmd_vel           是否直接发布 cmd_vel, 默认 False
  camera_fx/fy/cx/cy       相机内参 (px), 默认 Realsense D435 典型值
"""

import os
# 屏蔽 GTK 模块缺失警告 (libcanberra-gtk-module)
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
    """KCF 跟踪 + XY 坐标输出节点"""

    def __init__(self):
        super().__init__('kcf_tracker_model')

        # 声明参数 ------------------------------------------------------------
        # 初始 bbox (接受浮点, 内部转 int)
        self.declare_parameter('x1', 290.0)
        self.declare_parameter('y1', 160.0)
        self.declare_parameter('x2', 350.0)
        self.declare_parameter('y2', 200.0)

        # 跟随参数
        self.declare_parameter('target_dist', 0.6)

        # 线性 PID
        self.declare_parameter('linear_Kp', 1.5)
        self.declare_parameter('linear_Ki', 0.0)
        self.declare_parameter('linear_Kd', 1.0)  # 微分项放大噪声, 跟随场景不需要

        # 角度 PID
        self.declare_parameter('angular_Kp', 0.5)
        self.declare_parameter('angular_Ki', 0.0)
        self.declare_parameter('angular_Kd', 2.0)

        # 死区 & 限速
        self.declare_parameter('dist_deadzone', 0.10)      # 100mm
        self.declare_parameter('angle_deadzone', 0.10)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.35)

        # 开关
        self.declare_parameter('enable_cmd_vel', True)
        self.declare_parameter('show_display', False)  # 是否弹出本地显示窗口

        # 相机内参 (默认 D435)
        self.declare_parameter('camera_fx', 606.0)
        self.declare_parameter('camera_fy', 605.0)
        self.declare_parameter('camera_cx', 321.0)
        self.declare_parameter('camera_cy', 241.0)

        # YOLO 外部检测 (独立脚本, 通过文件通信, 不依赖 ROS)
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('yolo_frame_path', '/tmp/kcf_yolo_frame.jpg')
        self.declare_parameter('yolo_bbox_path', '/tmp/kcf_yolo_bbox.json')
        self.declare_parameter('yolo_interval', 30)  # 每 N 帧(约1s)写一次帧给 YOLO
        self._sync_params()
        # show_display 只读一次, 不放入 _sync_params
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value

        # 本地显示: 启动时一次性检测 GUI 后端是否可用
        self._first_frame_saved = False  # 首帧截图标记
        self._display_ok = False
        if self.show_display:
            try:
                cv2.namedWindow('_kcf_test', cv2.WINDOW_GUI_EXPANDED)
                cv2.destroyWindow('_kcf_test')
                self._display_ok = True
                self.get_logger().info('本地显示窗口已就绪')
            except cv2.error:
                self.get_logger().warn('本地显示不可用 (缺少 GUI 后端), 已跳过')

        # CV bridge
        self.bridge = CvBridge()

        # 跟踪器 (CSRT: 精度更高, 尺度适应更好, 不易丢目标)
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker_initialized = False

        # 状态变量
        self.result_bbox = None         # (x, y, w, h) 当前跟踪结果
        self.is_tracking = False        # 当前帧是否成功
        self.loss_start_time = None     # 丢失开始时间
        self.last_loss_log_time = 0.0   # 上次丢失日志时间 (限速用)
        self.rgb_image = None           # 最新 RGB 帧
        self.depth_image = None         # 最新深度帧

        # 图像尺寸 (初始化时获取)
        self.img_w = 640
        self.img_h = 480

        # 平滑状态 (低通滤波用)
        self._depth_smooth = 0.0           # 深度 EMA
        self._prev_bbox = None             # 上一帧 bbox
        self._init_bbox_area = 0           # 初始 bbox 面积 (模板防污染用)

        # 三模板恢复: 每个是 (rgb, edge, hist) 三元组
        self._tmpl_initial = None        # 初始: 第一帧, 永久保留
        self._tmpl_best = None           # 最优: 跟踪中质量最高的帧
        self._tmpl_recent = None         # 最近: 最近一次通过筛选的采集
        self._tmpl_best_quality = 0.0    # 最优模板质量分
        self._tmpl_last_capture = 0.0    # 上次尝试采集时间
        self._tracking_stable_since = 0.0 # 当前跟踪段开始时间 (筛选用)
        self._recovery_candidate = None  # (score, bbox) 候选匹配
        self._recovery_candidate_count = 0  # 连续稳定帧数

        self._content_mismatch_count = 0   # 颜色+纹理连续不匹配计数

        # 颜色直方图反投影 (自适应框)
        self._backproject_last = 0.0     # 上次反投影时间
        self._refined_bbox = None        # EMA 平滑后的框
        self._debug_last_publish = 0.0   # 上次调试图像发布时间

        # PID 控制器
        self._init_pid()

        # 订阅 (message_filters 时间同步)
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1
        )
        self.sync.registerCallback(self._sync_callback)

        # YOLO 文件桥接状态
        self._yolo_frame_count = 0
        self._yolo_last_bbox_mtime = 0.0
        self._yolo_bbox = None             # 最近一次 YOLO 检测 bbox (用于调试显示)

        # 发布者
        self.xy_pub = self.create_publisher(Point, '/kcf/tracked_xy', 10)
        self.bbox_pub = self.create_publisher(PolygonStamped, '/kcf/tracked_bbox', 10)
        self.debug_pub = self.create_publisher(Image, '/kcf/debug_image', 10)
        self.status_pub = self.create_publisher(Bool, '/kcf/tracking_status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 本地实时显示窗口
        self.display_name = 'KCF_Tracker'

        self.get_logger().info('KCF Tracker Model 节点已启动')

    # ------------------------------------------------------------------
    def _sync_params(self):
        """一次性读取全部参数"""
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

        # YOLO
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
        # 解码 RGB
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 解码失败: {e}')
            return

        # 解码深度 — 兼容 16UC1(mm) 和 32FC1(m) 两种编码
        try:
            if depth_msg.encoding == '16UC1':
                # 毫米转米
                raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                self.depth_image = raw.astype(np.float32) / 1000.0
            elif depth_msg.encoding in ('32FC1', 'TYPE_32FC1'):
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            else:
                # 自动尝试
                self.get_logger().warn(f'未知深度编码: {depth_msg.encoding}, 尝试 16UC1')
                raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                self.depth_image = raw.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'深度解码失败 (encoding={depth_msg.encoding}): {e}')
            return

        self.img_h, self.img_w = self.rgb_image.shape[:2]

        # 初始化追踪器
        if not self.tracker_initialized:
            self._init_tracker()

        # 运行跟踪
        self._run_tracking()

        # 计算并发布 XY 坐标
        self._publish_xy()

        # 发布 bbox
        self._publish_bbox()

        # 发布调试图像 (ROS topic, 节流1Hz)
        t = time.time()
        if t - self._debug_last_publish >= 1.0:
            self._publish_debug()
            self._debug_last_publish = t

        # 本地实时显示 + 首帧截图 (仅 show_display=true 时启用)
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

        # YOLO 文件桥接 (写帧 + 读 bbox)
        if self.use_yolo:
            self._yolo_bridge()

        # 可选: 发布 cmd_vel
        if self.enable_cmd_vel:
            self._publish_cmd_vel()

    # ------------------------------------------------------------------
    def _init_tracker(self):
        """用参数中指定的 bbox 初始化 CSRT"""
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
        self._init_bbox_area = w * h           # 记录初始面积
        self._init_all_templates()             # 三模板初始采集
        self._tracking_stable_since = time.time()
        self.get_logger().info(f'CSRT 已初始化, bbox=({self.x1},{self.y1}) ({w}x{h})')

    # ------------------------------------------------------------------
    def _yolo_bridge(self):
        """文件桥接: 写帧给 YOLO, 仅在丢失时读取 YOLO bbox 救援"""
        import json
        self._yolo_frame_count += 1

        # 每 yolo_interval 帧写一次 jpg + 当前状态
        if self._yolo_frame_count % self.yolo_interval == 0 and self.rgb_image is not None:
            cv2.imwrite(self.yolo_frame_path, self.rgb_image)
            # 始终写出 bbox (丢失时也用最后已知位置, 供 YOLO 做 IoU 匹配)
            state = {'tracking': self.is_tracking}
            if self.result_bbox is not None:
                bx, by, bw, bh = self.result_bbox
                state['bbox'] = {'x': int(bx), 'y': int(by),
                                 'w': int(bw), 'h': int(bh)}
            with open(self.yolo_frame_path + '.state.json', 'w') as f:
                json.dump(state, f)

        # 正常跟踪中不读取 YOLO, CSRT 自己做主
        if self.is_tracking:
            return

        # 目标已丢失, 检查 YOLO 是否有新的检测结果可以救援
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
            # 存储 YOLO bbox 用于调试显示
            self._yolo_bbox = (x1, y1, x2, y2)
        except Exception:
            return

        # YOLO 救援: 仅做位置检查, 接受后直接用 YOLO bbox 重建
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
        self._init_all_templates()
        self.get_logger().info(
            f'YOLO 救援成功 (丢失 {loss_elapsed:.0f}s), '
            f'CSRT 已重建: {bbox}')

    def _refine_bbox_color(self, bbox):
        """颜色反投影自适应框: 用初始 HSV 直方图在当前帧找完整颜色区域.
        返回 refined bbox (x,y,w,h) 或 None"""
        if self._tmpl_initial is None or self.rgb_image is None:
            return None
        x, y, w, h = [int(v) for v in bbox]
        # 扩展搜索区域: CSRT bbox 外扩 40%
        margin = 0.4
        ex = max(0, int(x - w * margin))
        ey = max(0, int(y - h * margin))
        ew = min(self.img_w - ex, int(w * (1 + 2 * margin)))
        eh = min(self.img_h - ey, int(h * (1 + 2 * margin)))
        if ew < 20 or eh < 20:
            return None
        roi = self.rgb_image[ey:ey + eh, ex:ex + ew]
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 反投影
        bp = cv2.calcBackProject([roi_hsv], [0, 1], self._tmpl_initial[2],
                                 [0, 180, 0, 256], 1)
        _, mask = cv2.threshold(bp, 60, 255, cv2.THRESH_BINARY)
        # 形态学闭合 + 开运算去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 找最大轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area_ratio = cv2.contourArea(largest) / (ew * eh)
        if area_ratio < 0.03 or area_ratio > 0.95:
            return None
        rx, ry, rw, rh = cv2.boundingRect(largest)
        # 转全图坐标
        fx, fy = ex + rx, ey + ry
        # 中心必须接近 CSRT 中心
        fc_x, fc_y = fx + rw / 2, fy + rh / 2
        csrt_cx, csrt_cy = x + w / 2, y + h / 2
        if abs(fc_x - csrt_cx) > w * 1.2 or abs(fc_y - csrt_cy) > h * 1.2:
            return None
        return (fx, fy, rw, rh)

    def _extract_template(self):
        """从当前 result_bbox 提取 (rgb, edge, hist) 三元组, 失败返回 None"""
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
        """初始化时三模板用同一帧填充"""
        tmpl = self._extract_template()
        if tmpl is not None:
            self._tmpl_initial = tmpl
            self._tmpl_best = tmpl
            self._tmpl_recent = tmpl
            self._tmpl_best_quality = 1.0
            self._tmpl_last_capture = time.time()

    def _try_capture_template(self):
        """尝试采集模板: 通过筛选条件则存入 recent, 质量超过则替 best"""
        if not self.is_tracking or self._tmpl_initial is None:
            return
        x, y, w, h = [int(v) for v in self.result_bbox]
        # 筛选1: bbox 面积 >= 初始 50%
        if self._init_bbox_area > 0 and w * h < self._init_bbox_area * 0.5:
            return
        # 筛选2: 跟踪稳定 >= 2 秒
        if time.time() - self._tracking_stable_since < 2.0:
            return
        # 提取模板
        tmpl = self._extract_template()
        if tmpl is None:
            return
        # 筛选3: 与初始模板颜色相似度 >= 0.6
        _, _, hist = tmpl
        init_hist = self._tmpl_initial[2]
        quality = cv2.compareHist(init_hist, hist, cv2.HISTCMP_CORREL)
        if quality < 0.6:
            return
        # 通过筛选: 存入 recent
        self._tmpl_recent = tmpl
        self._tmpl_last_capture = time.time()
        # 质量超过旧最优则替换
        if quality > self._tmpl_best_quality:
            self._tmpl_best = tmpl
            self._tmpl_best_quality = quality
            self.get_logger().info(f'模板更新: best quality={quality:.3f}')

    def _template_recovery(self):
        """三模板多尺度匹配: 遍历初始/最优/最近, 在最后已知位置附近搜索
        返回 (combined, bbox, (edge_score, color_score)) 或 None"""
        templates = [t for t in (self._tmpl_initial, self._tmpl_best, self._tmpl_recent)
                     if t is not None]
        if not templates:
            return None
        # 搜索窗口: 以最后已知 bbox 为中心, 半径 3x bbox
        if self.result_bbox is not None:
            lx, ly, lw, lh = [int(v) for v in self.result_bbox]
            lcx, lcy = lx + lw // 2, ly + lh // 2
            radius = max(lw, lh) * 3
            sx = max(0, lcx - radius)
            sy = max(0, lcy - radius)
            ex = min(self.img_w, lcx + radius)
            ey = min(self.img_h, lcy + radius)
            gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
            edge_img = cv2.Canny(gray, 50, 150)
            edge_img = edge_img[sy:ey, sx:ex]
            search_rgb = self.rgb_image[sy:ey, sx:ex]
            offset_x, offset_y = sx, sy
        else:
            gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
            edge_img = cv2.Canny(gray, 50, 150)
            search_rgb = self.rgb_image
            offset_x, offset_y = 0, 0

        scales = [0.6, 0.8, 1.0, 1.3, 1.7, 2.2]
        best_overall = None  # (score, bbox, edge, color, threshold, scale)
        global_best = -1.0   # 全局最高分, 用于日志

        for rgb_tmpl, edge_tmpl, _hist in templates:
            for s in scales:
                tw = int(edge_tmpl.shape[1] * s)
                th = int(edge_tmpl.shape[0] * s)
                if tw < 8 or th < 8 or tw > edge_img.shape[1] or th > edge_img.shape[0]:
                    continue
                # 尺度自适应阈值
                area = tw * th
                threshold = 0.40 + 0.20 * (2500.0 / max(area, 64)) ** 0.5
                threshold = max(0.35, min(0.80, threshold))
                # 边缘匹配
                resized_edge = cv2.resize(edge_tmpl, (tw, th), interpolation=cv2.INTER_NEAREST)
                res_e = cv2.matchTemplate(edge_img, resized_edge, cv2.TM_CCOEFF_NORMED)
                _, mv_e, _, ml_e = cv2.minMaxLoc(res_e)
                # 颜色验证: 同位置 RGB CCORR
                rgb_tmpl_s = cv2.resize(rgb_tmpl, (tw, th), interpolation=cv2.INTER_AREA)
                res_c = cv2.matchTemplate(search_rgb, rgb_tmpl_s, cv2.TM_CCORR_NORMED)
                mv_c = res_c[ml_e[1], ml_e[0]]
                # 综合得分: 几何平均
                combined = (mv_e * mv_c) ** 0.5
                if combined > global_best:
                    global_best = combined
                if combined > threshold:
                    if best_overall is None or combined > best_overall[0]:
                        bw = min(int(tw), edge_img.shape[1] - int(ml_e[0]))
                        bh = min(int(th), edge_img.shape[0] - int(ml_e[1]))
                        if bw >= 8 and bh >= 8:
                            # 转全图坐标
                            bx = int(ml_e[0]) + offset_x
                            by_ = int(ml_e[1]) + offset_y
                            bbox = (bx, by_, bw, bh)
                            best_overall = (combined, bbox, mv_e, mv_c, threshold, s)

        if best_overall is not None:
            score, bbox, edge_s, color_s, thr, s = best_overall
            self.get_logger().info(
                f'模板恢复: s={s:.1f}x pos=({bbox[0]},{bbox[1]}) size={bbox[2]}x{bbox[3]} '
                f'thr={thr:.2f} edge={edge_s:.3f} color={color_s:.3f} '
                f'combined={score:.3f} ✓')
            return (score, bbox, (edge_s, color_s))
        else:
            self.get_logger().info(
                f'模板恢复: 无匹配 (最高 combined={global_best:.3f})')
            return None

    # ------------------------------------------------------------------
    def _run_tracking(self):
        """CSRT 跟踪 + 丢失后模板匹配恢复"""
        if not self.tracker_initialized:
            return

        now_t = time.time()

        # --- 丢失状态: 冻结 CSRT, 模板匹配恢复 (需连续稳定确认) ---
        if not self.is_tracking:
            recovered = self._template_recovery()
            if recovered is not None:
                score, r_bbox, (edge_s, color_s) = recovered
                rx, ry, rw, rh = r_bbox
                rc_x, rc_y = rx + rw / 2.0, ry + rh / 2.0
                if self._recovery_candidate is not None:
                    _, prev_bbox = self._recovery_candidate
                    px, py, pw, ph = prev_bbox
                    pc_x, pc_y = px + pw / 2.0, py + ph / 2.0
                    dist = np.hypot(rc_x - pc_x, rc_y - pc_y)
                    if dist < 30.0:
                        self._recovery_candidate_count += 1
                        self._recovery_candidate = (score, r_bbox)
                        if self._recovery_candidate_count >= 3:
                            self.tracker = cv2.TrackerCSRT_create()
                            self.tracker.init(self.rgb_image, r_bbox)
                            self.tracker_initialized = True
                            self.result_bbox = r_bbox
                            self._prev_bbox = None
                            self._init_bbox_area = int(rw * rh)
                            self.is_tracking = True
                            self.loss_start_time = None
                            self._recovery_candidate = None
                            self._recovery_candidate_count = 0
                            self._refined_bbox = None
                            self._tracking_stable_since = now_t
                            self._init_all_templates()
                            self.get_logger().info(
                                f'模板恢复成功 (e={edge_s:.2f} c={color_s:.2f} '
                                f'combined={score:.2f}, 稳定3帧)')
                            self.status_pub.publish(Bool(data=True))
                    else:
                        self._recovery_candidate = (score, r_bbox)
                        self._recovery_candidate_count = 1
                else:
                    self._recovery_candidate = (score, r_bbox)
                    self._recovery_candidate_count = 1
            else:
                self._recovery_candidate = None
                self._recovery_candidate_count = 0
                elapsed = now_t - self.loss_start_time if self.loss_start_time else 0
                if now_t - self.last_loss_log_time >= 1.0:
                    self.last_loss_log_time = now_t
                    self.get_logger().warn(f'目标丢失中, {elapsed:.0f}s')
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

        if size_ok and jump_ok:
            self.result_bbox = bbox
            self._prev_bbox = bbox
            self.is_tracking = True
            self.loss_start_time = None
            self.last_loss_log_time = now_t
            # 颜色反投影自适应框: 反投影直接给出完整位置+大小
            if now_t - self._backproject_last >= 0.5:
                self._backproject_last = now_t
                # --- 内容校验 + 反投影: 优先用上次反投影修正后的框, 避免 CSRT 漂移干扰 ---
                vb = self._refined_bbox if self._refined_bbox is not None else bbox
                vx, vy, vw, vh = [int(v) for v in vb]
                cx, cy = max(0, vx), max(0, vy)
                cw = min(vw, self.img_w - cx)
                ch = min(vh, self.img_h - cy)
                if cw <= 10 or ch <= 10:
                    self._content_mismatch_count += 1
                else:
                    roi = self.rgb_image[cy:cy+ch, cx:cx+cw]
                    # 颜色
                    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    roi_hist = cv2.calcHist(
                        [roi_hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
                    cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
                    color_sim = cv2.compareHist(
                        self._tmpl_initial[2], roi_hist, cv2.HISTCMP_CORREL)
                    # 纹理
                    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    roi_edge = cv2.Canny(roi_gray, 50, 150)
                    roi_edge_rs = cv2.resize(roi_edge,
                        (self._tmpl_initial[1].shape[1], self._tmpl_initial[1].shape[0]),
                        interpolation=cv2.INTER_NEAREST)
                    edge_sim = cv2.matchTemplate(
                        roi_edge_rs, self._tmpl_initial[1], cv2.TM_CCOEFF_NORMED)[0][0]
                    # 双重判断: 颜色和纹理都低才计为不匹配
                    if color_sim < 0.3 and edge_sim < 0.2:
                        self._content_mismatch_count += 1
                        if self._content_mismatch_count >= 6:
                            self.get_logger().warn(
                                f'目标丢失: 内容不匹配 '
                                f'(color={color_sim:.2f} edge={edge_sim:.2f})')
                            self.cmd_pub.publish(Twist())
                            self.is_tracking = False
                            self.loss_start_time = now_t
                            self.last_loss_log_time = now_t
                            self._content_mismatch_count = 0
                            self._prev_bbox = None
                            self._refined_bbox = None
                            self._recovery_candidate = None
                            self._recovery_candidate_count = 0
                            self.status_pub.publish(Bool(data=False))
                            return
                    else:
                        self._content_mismatch_count = 0
                # --- 反投影自适应框 ---
                refined = self._refine_bbox_color(vb)
                if refined is not None:
                    self.result_bbox = refined
                    self._prev_bbox = refined
                    self._refined_bbox = refined
                    csrt_a = w * h
                    refined_a = refined[2] * refined[3]
                    if refined_a > csrt_a * 1.2:
                        # 反投影框明显大于 CSRT → 重建 CSRT
                        self.tracker = cv2.TrackerCSRT_create()
                        self.tracker.init(self.rgb_image, refined)
                        self._init_bbox_area = refined_a
                        self._tracking_stable_since = time.time()
                else:
                    self._refined_bbox = None  # 反投影失败, 退回 CSRT
            # 跨帧覆盖: 反投影框持续修正输出, 阻止 CSRT 漂移
            if self._refined_bbox is not None:
                self.result_bbox = self._refined_bbox
                self._prev_bbox = self._refined_bbox
            # 定期模板采集 (每3秒尝试, 通过筛选才更新)
            if now_t - self._tmpl_last_capture >= 3.0:
                self._try_capture_template()
        else:
            if self.is_tracking:
                reason = []
                if not size_ok:
                    reason.append(f'bbox比例异常({bbox_ratio:.3f})')
                if not jump_ok:
                    reason.append('中心跳变过大')
                self.get_logger().warn(
                    f'目标丢失: {", ".join(reason)}')
                self.cmd_pub.publish(Twist())
            self.is_tracking = False
            self.loss_start_time = now_t
            self.last_loss_log_time = now_t
            self._refined_bbox = None
            self._content_mismatch_count = 0

        self.status_pub.publish(Bool(data=self.is_tracking))

    # ------------------------------------------------------------------
    def _get_depth(self) -> float:
        """5 点采样 → 中位数 → EMA 平滑, 返回米"""
        if self.depth_image is None or self.result_bbox is None:
            return -1.0

        x, y, w, h = self.result_bbox
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

        # 中位数抗异常值, 再做 EMA 平滑
        raw = float(np.median(valid))
        alpha = 0.8
        if self._depth_smooth <= 0:
            self._depth_smooth = raw  # 首次直接赋值
        else:
            self._depth_smooth = alpha * raw + (1.0 - alpha) * self._depth_smooth
        return self._depth_smooth

    # ------------------------------------------------------------------
    def _pixel_to_world(self, px: float, py: float, depth: float):
        """像素坐标 -> 真实 XY 坐标 (以相机光心为原点)"""
        X = (px - self.camera_cx) * depth / self.camera_fx
        Y = (py - self.camera_cy) * depth / self.camera_fy
        return X, Y

    # ------------------------------------------------------------------
    def _publish_xy(self):
        """发布目标真实 XY 坐标"""
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
        """发布图像边界框"""
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
        """发布带跟踪框的调试图像"""
        if self.rgb_image is None:
            return

        debug = self.rgb_image.copy()

        if self.result_bbox is not None:
            x, y, w, h = [int(v) for v in self.result_bbox]
            color = (0, 255, 255) if self.is_tracking else (0, 0, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            cv2.circle(debug, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)

            # 显示深度
            d = self._get_depth()
            if d > 0:
                cv2.putText(debug, f'{d:.2f}m', (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # YOLO 检测 bbox (绿色虚线)
        if self._yolo_bbox is not None:
            yx1, yy1, yx2, yy2 = self._yolo_bbox
            cv2.rectangle(debug, (yx1, yy1), (yx2, yy2), (0, 255, 0), 1)
            cv2.putText(debug, 'YOLO', (yx1, yy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # 状态文字
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
        """构建显示图像: RGB + 深度伪彩色 + 调试信息叠加, 返回 composite 图像"""
        if self.rgb_image is None:
            return None

        # 上半部分: RGB 跟踪画面
        rgb_view = self.rgb_image.copy()
        if self.result_bbox is not None:
            x, y, w, h = [int(v) for v in self.result_bbox]
            bbox_color = (0, 255, 255) if self.is_tracking else (0, 0, 255)
            cv2.rectangle(rgb_view, (x, y), (x + w, y + h), bbox_color, 2)
            cv2.circle(rgb_view, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)
            cv2.line(rgb_view, (self.img_w // 2, 0), (self.img_w // 2, self.img_h),
                     (255, 255, 255), 1)  # 中心十字线
            cv2.line(rgb_view, (0, self.img_h // 2), (self.img_w, self.img_h // 2),
                     (255, 255, 255), 1)

        # YOLO 检测 bbox (绿色虚线, 标 YOLO 标签)
        if self._yolo_bbox is not None:
            yx1, yy1, yx2, yy2 = self._yolo_bbox
            cv2.rectangle(rgb_view, (yx1, yy1), (yx2, yy2), (0, 255, 0), 1)
            cv2.putText(rgb_view, 'YOLO', (yx1, yy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # 下半部分: 深度伪彩色图 (自适应范围, 取有效深度的 5%~95% 分位)
        depth_view = None
        if self.depth_image is not None:
            valid_d = self.depth_image[(self.depth_image > 0.2) & (self.depth_image < 10.0)]
            if valid_d.size > 100:
                d_min = max(0.2, float(np.percentile(valid_d, 5)))
                d_max = min(6.0, float(np.percentile(valid_d, 95)))
            else:
                d_min, d_max = 0.2, 6.0  # 无有效数据时回退
            # 确保最小跨度 1m
            if d_max - d_min < 1.0:
                d_max = d_min + 1.0
            depth_clip = np.clip(self.depth_image, d_min, d_max)
            depth_norm = ((depth_clip - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            depth_view = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

            # 深度图上也画框
            if self.result_bbox is not None:
                x, y, w, h = [int(v) for v in self.result_bbox]
                cv2.rectangle(depth_view, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.circle(depth_view, (x + w // 2, y + h // 2), 3, (0, 0, 255), -1)

        # 拼接上下画面
        if depth_view is not None:
            # 缩放到与 RGB 等宽
            if depth_view.shape[1] != rgb_view.shape[1]:
                depth_view = cv2.resize(depth_view, (rgb_view.shape[1],
                                        int(rgb_view.shape[1] * depth_view.shape[0] / depth_view.shape[1])))
            display = np.vstack([rgb_view, depth_view])
        else:
            display = rgb_view

        # --- 叠加调试信息面板 (左上角半透明) ---
        overlay = display.copy()
        panel_w, panel_h = 280, 190
        cv2.rectangle(overlay, (5, 5), (5 + panel_w, 5 + panel_h), (30, 30, 30), -1)
        display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        row_y = 25
        line_h = 18

        # 跟踪状态
        status_text = 'TRACKING' if self.is_tracking else 'LOST'
        status_color = (0, 255, 0) if self.is_tracking else (0, 0, 255)
        cv2.putText(display, f'Status: {status_text}', (12, row_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 1)
        row_y += line_h

        # bbox 位置
        if self.result_bbox is not None:
            bx, by, bw, bh = self.result_bbox
            cv2.putText(display, f'BBox: ({int(bx)},{int(by)}) {int(bw)}x{int(bh)}', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        row_y += line_h

        # 深度 & XY 真实坐标
        depth = self._get_depth()
        if depth > 0 and self.result_bbox is not None:
            _x, _y, _w, _h = self.result_bbox
            cx = _x + _w / 2.0
            cy = _y + _h / 2.0
            rx, ry = self._pixel_to_world(cx, cy, depth)
            cv2.putText(display, f'Depth: {depth:.3f}m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h
            # 距离误差 (死区内绿色, 超出青色)
            dist_err = depth - self.target_dist
            err_color = (0, 255, 0) if abs(dist_err) < self.dist_deadzone else (0, 200, 255)
            cv2.putText(display, f'Dist Error: {dist_err:+.3f}m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, err_color, 1)
            row_y += line_h
            cv2.putText(display, f'World XY: ({rx:+.3f}, {ry:+.3f})m', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h
        else:
            # 深度无效时也显示，方便排查
            cv2.putText(display, f'Depth: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h
            cv2.putText(display, f'Dist Error: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h
            cv2.putText(display, f'World XY: N/A', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            row_y += line_h

        # PID 输出 (仅在 enable_cmd_vel 时有效)
        if self.enable_cmd_vel and self.is_tracking:
            cv2.putText(display, f'Linear PID: {self.linear_pid.targetpoint:+.3f}', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            row_y += line_h

        # 目标距离
        cv2.putText(display, f'Target Dist: {self.target_dist:.2f}m', (12, row_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        row_y += line_h

        # 丢失计时器
        if self.loss_start_time is not None:
            elapsed = time.time() - self.loss_start_time
            cv2.putText(display, f'Loss time: {elapsed:.1f}s', (12, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # 缩放显示 (限制最大 1280 宽)
        max_w = 1280
        if display.shape[1] > max_w:
            scale = max_w / display.shape[1]
            display = cv2.resize(display, (max_w, int(display.shape[0] * scale)))

        return display

    # ------------------------------------------------------------------
    def _publish_cmd_vel(self):
        """计算并发布 cmd_vel: EMA 平滑 + 转向减速 + 深度无效零速"""
        if self.result_bbox is None or not self.is_tracking:
            return

        depth = self._get_depth()
        if depth <= 0:
            self.cmd_pub.publish(Twist())  # 深度无效 → 停车
            return

        x, y, w, h = self.result_bbox
        cx = x + w / 2.0

        # --- 线性速度 ---
        linear_raw = -self.linear_pid.compute(self.target_dist, depth)

        if abs(depth - self.target_dist) < self.dist_deadzone:
            linear_raw = 0.0

        linear_raw = max(-0.3, min(self.max_linear_speed, linear_raw))

        # --- 角速度: 简单比例控制, 像素偏移归一化到 [-1,1] ---
        angle_ratio = (self.img_w / 2.0 - cx) / (self.img_w / 2.0)
        angular_raw = self.angular_Kp * angle_ratio
        angular_raw = max(-self.max_angular_speed, min(self.max_angular_speed, angular_raw))
        if abs(angular_raw) < self.angle_deadzone:
            angular_raw = 0.0

        # --- 转向自动减速 ---
        angle_ratio = abs(angular_raw) / self.max_angular_speed if self.max_angular_speed > 0 else 0.0
        linear_raw *= (1.0 - 0.6 * min(angle_ratio, 1.0))

        # --- 发布 ---
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
