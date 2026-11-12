"""Rigid transforms use T_parent_child; quaternions use x, y, z, w."""
import numpy as np


def rigid(value):
    a = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(a).all() or not np.allclose(a[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(a[:3, :3].T @ a[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(a[:3, :3]), 1., atol=1e-5)):
        raise ValueError('extrinsic must be a finite SE(3) transform')
    return a


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., atol=.001):
        raise ValueError('quaternion must have unit norm')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(r):
    # Symmetric eigenproblem is stable at rotations close to pi as well.
    m = np.asarray(r)
    k = np.array([[m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]],
                  [m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]],
                  [m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                  [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3.
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def pose_matrix(p, q):
    t = np.eye(4)
    t[:3, :3] = rotation(q)
    t[:3, 3] = p
    return rigid(t)
