"""Wall-paced, original-timestamp EuRoC replay and raw estimator evidence capture."""
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np
from .dataset import load_dataset
from .geometry import quaternion
from .rectification import StereoRectifier
from .runtime import CAMERA_FRAMES, FRAME, events, prepare_run, stamp_parts
from .trajectory import Pose, save_evaluation


def main(argv=None):
    # Keep all middleware imports here: validation, geometry and metrics run without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Image, CameraInfo, Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import OdomInfo
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    class Player(Node):
        def __init__(self):
            super().__init__('euroc_player')
            defaults = dict(dataset='', output_dir='', prepared=False, mode='vio', run_estimator=False,
                            readiness_timeout_s=90., drain_seconds=5.)
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            def parameter(key):
                return self.get_parameter(key).value
            self.mode = parameter('mode')
            self.run_estimator = parameter('run_estimator')
            self.ready_timeout = float(parameter('readiness_timeout_s'))
            self.drain_seconds = float(parameter('drain_seconds'))
            if self.mode not in ('stereo', 'vio') or not (0 < self.ready_timeout < 3600) or not (0 <= self.drain_seconds < 3600):
                raise ValueError('invalid mode, readiness timeout or drain duration')
            self.output = Path(parameter('output_dir')).expanduser().resolve()
            if parameter('prepared'):
                manifest = json.loads((self.output / 'manifest.json').read_text(encoding='utf-8'))
                self.data = load_dataset(parameter('dataset'))
                self.rect = StereoRectifier(self.data)
                if (Path(manifest['data']['mav0']) != self.data.root or manifest['mode'] != self.mode
                        or manifest['run_estimator'] != self.run_estimator):
                    raise ValueError('prepared manifest does not match player parameters')
            else:
                if self.run_estimator:
                    raise ValueError('run_estimator requires euroc.launch.py (it creates the VINS process)')
                self.data, self.rect, _ = prepare_run(parameter('dataset'), self.output, self.mode, False)
            self.streams = []
            def writer(name, fields):
                stream = (self.output / name).open('x', encoding='utf-8', newline='')
                self.streams.append(stream)
                value = csv.writer(stream)
                value.writerow(fields)
                return value
            self.odom_csv = writer('trajectory.csv', ['timestamp_ns', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
                'frame_id', 'child_frame_id', 'receive_monotonic_ns', 'data_age_ns',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'pose_covariance', 'twist_covariance'])
            self.info_csv = writer('visual_status.csv', ['timestamp_ns', 'receive_monotonic_ns', 'data_age_ns',
                'lost', 'inliers', 'features', 'matches', 'time_estimation_s', 'time_particle_filtering_s'])
            self.timing_csv = writer('playback.csv', ['timestamp_ns', 'kind', 'dispatch_monotonic_ns',
                'schedule_lateness_ns', 'rectification_ns'])
            self.poses, self.invalid_output = [], []
            self.odom_count = self.info_count = self.stereo_count = self.imu_count = 0
            self.visual_lost = 0
            self.start_wall_ns = self.first_ns = self.last_clock_ns = None
            qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE)
            self.images, self.infos = [], []
            for side in ('left', 'right'):
                self.images.append(self.create_publisher(Image, f'/benchmark/{side}/image_rect', qos))
                self.infos.append(self.create_publisher(CameraInfo, f'/benchmark/{side}/camera_info', qos))
            self.imu_pub = self.create_publisher(Imu, '/benchmark/imu', qos)
            self.clock_pub = self.create_publisher(Clock, '/clock', QoSProfile(depth=10))
            self.create_subscription(Odometry, '/benchmark/odom', self.on_odom, qos)
            self.create_subscription(OdomInfo, '/benchmark/odom_info', self.on_info, qos)
            self.tf = StaticTransformBroadcaster(self)
            self.static_transforms()

        def stamp(self, header, ns):
            header.stamp.sec, header.stamp.nanosec = stamp_parts(ns)

        def static_transforms(self):
            values = []
            first = min(self.data.imu.times[0], self.data.cameras[0].times[0])
            def add(parent, child, t):
                msg = TransformStamped()
                msg.header.frame_id, msg.child_frame_id = parent, child
                self.stamp(msg.header, first)
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = t[:3, 3].tolist()
                q = quaternion(t[:3, :3]).tolist()
                (msg.transform.rotation.x, msg.transform.rotation.y,
                 msg.transform.rotation.z, msg.transform.rotation.w) = q
                values.append(msg)
            add(FRAME, 'benchmark_imu', np.eye(4))
            for side, camera in enumerate(self.data.cameras):
                raw = f'benchmark_cam{side}_raw'
                add(FRAME, raw, camera.body_T_sensor)
                t = np.eye(4)
                t[:3, :3] = self.rect.R[side].T
                add(raw, CAMERA_FRAMES[side], t)
            self.tf.sendTransform(values)

        def clock(self, ns):
            ns = max(ns, self.last_clock_ns if self.last_clock_ns is not None else ns)
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = stamp_parts(ns)
            self.clock_pub.publish(msg)
            self.last_clock_ns = ns

        def age(self, stamp):
            if self.start_wall_ns is None:
                return None
            return self.first_ns + time.monotonic_ns() - self.start_wall_ns - stamp

        def ns(self, header):
            return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

        def on_odom(self, msg):
            ns = self.ns(msg.header)
            p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
            self.odom_csv.writerow([ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                msg.header.frame_id, msg.child_frame_id, time.monotonic_ns(), self.age(ns),
                v.x, v.y, v.z, w.x, w.y, w.z,
                json.dumps(list(msg.pose.covariance)), json.dumps(list(msg.twist.covariance))])
            self.odom_count += 1
            try:
                if msg.header.frame_id != 'benchmark_odom' or msg.child_frame_id != FRAME:
                    raise ValueError('odom frame must be benchmark_odom and child base_footprint (IMU origin)')
                pose = Pose(ns, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
                pose.matrix()
                if self.poses and ns <= self.poses[-1].t:
                    raise ValueError('odom timestamps are not strictly increasing')
                self.poses.append(pose)
            except ValueError as exc:
                self.invalid_output.append(dict(timestamp_ns=ns, reason=str(exc)))

        def on_info(self, msg):
            ns = self.ns(msg.header)
            self.info_count += 1
            self.visual_lost += int(msg.lost)
            self.info_csv.writerow([ns, time.monotonic_ns(), self.age(ns), msg.lost,
                msg.inliers, msg.features, msg.matches, msg.time_estimation, msg.time_particle_filtering])

        def ready(self):
            if not self.run_estimator:
                return True
            pubs = [*self.images, *self.infos]
            if self.mode == 'vio':
                pubs.append(self.imu_pub)
            return (all(pub.get_subscription_count() > 0 for pub in pubs)
                    and self.count_publishers('/benchmark/odom_info') > 0)

        def publish_sample(self, ns, kind, index):
            rectify_ns = 0
            if kind == 'imu':
                msg = Imu()
                self.stamp(msg.header, ns)
                msg.header.frame_id = 'benchmark_imu'
                # No orientation estimator exists here. Zero angular/linear covariance
                # means unknown in sensor_msgs/Imu, never perfect measurements.
                msg.orientation_covariance[0] = -1.
                row = self.data.imu.values[index]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = row[:3].tolist()
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = row[3:].tolist()
                self.imu_pub.publish(msg)
                self.imu_count += 1
            else:
                start = time.monotonic_ns()
                pixels = [self.rect.image(self.data.cameras[side].paths[index], side) for side in range(2)]
                rectify_ns = time.monotonic_ns() - start
                for side, image in enumerate(pixels):
                    msg = Image()
                    self.stamp(msg.header, ns)
                    msg.header.frame_id = CAMERA_FRAMES[side]
                    msg.height, msg.width = image.shape
                    msg.encoding, msg.is_bigendian, msg.step = 'mono8', 0, image.shape[1]
                    msg.data = image.tobytes()
                    info = CameraInfo()
                    info.header = msg.header
                    for key, value in self.rect.camera_info(side).items():
                        setattr(info, key, value)
                    self.infos[side].publish(info)
                    self.images[side].publish(msg)
                self.stereo_count += 1
            now = time.monotonic_ns()
            self.timing_csv.writerow([ns, kind, now, now - self.start_wall_ns - (ns-self.first_ns), rectify_ns])

        def run(self):
            ready_deadline = time.monotonic() + self.ready_timeout
            self.clock(min(self.data.imu.times[0], self.data.cameras[0].times[0]))
            while rclpy.ok() and not self.ready():
                if time.monotonic() > ready_deadline:
                    raise RuntimeError('estimator sensor subscriptions/odom_info publisher not ready before timeout')
                rclpy.spin_once(self, timeout_sec=.05)
            if not rclpy.ok():
                raise RuntimeError('interrupted before replay')
            # Allow TF delivery after discovery, while still withholding sensor data.
            settle = time.monotonic() + .5
            while rclpy.ok() and time.monotonic() < settle:
                rclpy.spin_once(self, timeout_sec=.01)
            schedule = iter(events(self.data))
            item = next(schedule, None)
            self.first_ns = item[0]
            self.start_wall_ns = time.monotonic_ns()
            last_clock_wall = 0
            while rclpy.ok() and item is not None:
                now = time.monotonic_ns()
                elapsed = now - self.start_wall_ns
                due = item[0] - self.first_ns
                if now - last_clock_wall >= 5_000_000:
                    self.clock(self.first_ns + elapsed)
                    last_clock_wall = now
                if elapsed >= due:
                    self.clock(max(item[0], self.first_ns + elapsed))
                    self.publish_sample(*item)
                    item = next(schedule, None)
                    rclpy.spin_once(self, timeout_sec=0.)
                else:
                    rclpy.spin_once(self, timeout_sec=min(.001, (due-elapsed)/1e9))
            if not rclpy.ok():
                raise RuntimeError('interrupted during replay')
            deadline = time.monotonic() + self.drain_seconds
            while rclpy.ok() and time.monotonic() < deadline:
                self.clock(self.first_ns + time.monotonic_ns() - self.start_wall_ns)
                rclpy.spin_once(self, timeout_sec=.005)

        def finish(self, failure=None):
            for stream in self.streams:
                stream.flush()
                stream.close()
            if self.invalid_output:
                failure = failure or 'invalid estimator frames/poses/timestamps; inspect runtime.json and raw trajectory'
            report = dict(schema_version=1, failure=failure, invalid_output=self.invalid_output,
                          stereo_published=self.stereo_count, imu_published=self.imu_count,
                          odom_samples=self.odom_count, visual_status_samples=self.info_count,
                          visual_lost_samples=self.visual_lost, estimator_enabled=self.run_estimator,
                          imu_covariance='orientation unavailable (-1); gyro/accel unknown (zeros, ROS convention)',
                          timing='monotonic dispatch lateness; receive data_age uses 1x replay epoch, includes queue delay',
                          completion='failed' if failure else 'stream_complete')
            (self.output / 'runtime.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
            return save_evaluation(self.data, self.poses, self.output, failure=failure)

    rclpy.init(args=argv)
    player, failure, score = None, None, None
    try:
        player = Player()
        player.run()
    except KeyboardInterrupt:
        failure = 'interrupted'
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print(f'euroc_player: {failure}', file=sys.stderr)
    finally:
        if player is not None:
            try:
                score = player.finish(failure)
            finally:
                player.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    failed_score = player is not None and player.run_estimator and score and score['status'] in ('failed', 'no_output')
    return 1 if failure or failed_score else 0


if __name__ == '__main__':
    raise SystemExit(main())
