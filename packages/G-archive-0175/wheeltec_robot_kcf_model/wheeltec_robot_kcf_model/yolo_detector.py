#!/usr/bin/env python3
"""
YOLO 低速检测 — 独立脚本, 在 conda 环境直接运行, 不依赖 ROS.

用法:
  conda activate <your-env>
  pip install ultralytics
  python3 yolo_detector.py --class person --interval 0.3

文件通信:
  读取  /tmp/kcf_yolo_frame.jpg    (KCF 节点写入的最新帧)
  写入  /tmp/kcf_yolo_bbox.json    (检测到的最大 bbox)
"""

import json
import time
import os
import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

# COCO 类别映射
COCO_CLASSES = {
    'person': 0, 'bicycle': 1, 'car': 2, 'motorcycle': 3,
    'bus': 5, 'truck': 7, 'cat': 15, 'dog': 16,
    'backpack': 24, 'suitcase': 28, 'bottle': 39, 'cup': 41,
    'chair': 56, 'couch': 57, 'laptop': 63, 'cell phone': 67,
}

FRAME_PATH = '/tmp/kcf_yolo_frame.jpg'
STATE_PATH = '/tmp/kcf_yolo_frame.jpg.state.json'
BBOX_PATH = '/tmp/kcf_yolo_bbox.json'
IOU_THRESH = 0.3  # IoU 阈值, 低于此值认为不是同一目标


def main():
    parser = argparse.ArgumentParser(description='YOLO detector for KCF tracker')
    parser.add_argument('--model', default='yolo11n.pt', help='YOLO model path')
    parser.add_argument('--class', dest='cls', default='all', help='Target COCO class (all=不限制)')
    parser.add_argument('--conf', type=float, default=0.5, help='Confidence threshold')
    parser.add_argument('--interval', type=float, default=0.3, help='Detection interval (s)')
    parser.add_argument('--frame', default=FRAME_PATH, help='Input frame path')
    parser.add_argument('--bbox', default=BBOX_PATH, help='Output bbox path')
    args = parser.parse_args()

    target_ids = None if args.cls == 'all' else [COCO_CLASSES.get(args.cls, 0)]
    print(f'[YOLO] Loading model: {args.model}')
    model = YOLO(args.model)
    cls_label = 'all COCO classes' if target_ids is None else f'{args.cls} (id={target_ids[0]})'
    print(f'[YOLO] Target: {cls_label}, interval={args.interval}s')
    print(f'[YOLO] Watching: {args.frame}')

    last_mtime = 0

    while True:
        # 等待新帧
        try:
            mtime = os.path.getmtime(args.frame)
        except OSError:
            time.sleep(args.interval)
            continue

        if mtime <= last_mtime:
            time.sleep(0.05)
            continue
        last_mtime = mtime

        # 读取图像
        img = cv2.imread(args.frame)
        if img is None:
            continue

        # 读取 CSRT 当前 bbox 和跟踪状态 (用于 IoU 匹配)
        csrt_bbox = None
        csrt_tracking = True  # 默认认为在跟踪中(保守)
        try:
            with open(STATE_PATH, 'r') as f:
                state = json.load(f)
            bb = state.get('bbox')
            if bb:
                csrt_bbox = (bb['x'], bb['y'], bb['x'] + bb['w'], bb['y'] + bb['h'])
            csrt_tracking = state.get('tracking', True)
        except Exception:
            pass

        # YOLO track 模式 (带 ID 一致性)
        results = model.track(img, conf=args.conf, classes=target_ids,
                              persist=True, verbose=False)

        # IoU 匹配: 只保留与 CSRT 当前框有重叠的检测
        best = None
        best_conf = 0

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [float(v) for v in xyxy]

                # IoU 匹配: 仅在 CSRT 跟踪正常时要求重叠; 丢失时放宽为最高置信度
                if csrt_tracking and csrt_bbox is not None:
                    iou = _compute_iou(csrt_bbox, (x1, y1, x2, y2))
                    if iou < IOU_THRESH:
                        continue

                if conf > best_conf:
                    best_conf = conf
                    best = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'conf': conf}

        # 写入 bbox
        if best is not None:
            data = {'bbox': best, 'timestamp': time.time()}
            tmp = args.bbox + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, args.bbox)  # 原子写入

        time.sleep(args.interval)


def _compute_iou(a, b):
    """两个 (x1,y1,x2,y2) 框的 IoU"""
    xa = max(a[0], b[0]); ya = max(a[1], b[1])
    xb = min(a[2], b[2]); yb = min(a[3], b[3])
    inter = max(0, xb - xa) * max(0, yb - ya)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter + 1e-6)


if __name__ == '__main__':
    main()
