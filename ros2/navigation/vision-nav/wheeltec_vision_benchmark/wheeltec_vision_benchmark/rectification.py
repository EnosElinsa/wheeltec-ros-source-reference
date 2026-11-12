"""OpenCV stereo geometry in the physical IMU/body coordinate system."""
import cv2
import numpy as np


class StereoRectifier:
    def __init__(self, data):
        left, right = data.cameras
        right_T_left = np.linalg.inv(right.body_T_sensor) @ left.body_T_sensor
        r0, r1, p0, p1, self.Q, _, _ = cv2.stereoRectify(
            left.K, left.D, right.K, right.D, left.size,
            right_T_left[:3, :3].copy(), right_T_left[:3, 3].reshape(3, 1).copy(),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        if abs(p1[1, 3]) > 1e-6 or p1[0, 3] >= 0:
            raise ValueError('VINS requires horizontal stereo with right camera on positive rectified x')
        self.R, self.P = (r0, r1), (p0, p1)
        self.maps, self.body_T_rect = [], []
        self.size = left.size
        for i, cam in enumerate(data.cameras):
            self.maps.append(cv2.initUndistortRectifyMap(cam.K, cam.D, self.R[i], self.P[i][:3, :3],
                                                       cam.size, cv2.CV_32FC1))
            sensor_T_rect = np.eye(4)
            sensor_T_rect[:3, :3] = self.R[i].T
            self.body_T_rect.append(cam.body_T_sensor @ sensor_T_rect)

    def image(self, path, side):
        raw = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if raw is None or (raw.shape[1], raw.shape[0]) != self.size:
            raise ValueError(f'image cannot be decoded or has wrong resolution: {path}')
        return cv2.remap(raw, *self.maps[side], interpolation=cv2.INTER_LINEAR)

    def camera_info(self, side):
        # Published pixels and optical frame are already rectified: D=0, R=I.
        return dict(width=self.size[0], height=self.size[1], distortion_model='plumb_bob',
                    d=[0.] * 5, k=self.P[side][:3, :3].ravel().tolist(),
                    r=np.eye(3).ravel().tolist(), p=self.P[side].ravel().tolist())
