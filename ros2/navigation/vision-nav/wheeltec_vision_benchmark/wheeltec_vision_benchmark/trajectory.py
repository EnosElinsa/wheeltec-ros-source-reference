"""Independent SE(3) metrics, with one-to-one nearest-time matching and no scale fit."""
import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from .dataset import load_dataset, timestamp
from .geometry import pose_matrix, quaternion


@dataclass
class Pose:
    t: int
    p: np.ndarray
    q: np.ndarray  # xyzw

    def matrix(self):
        return pose_matrix(self.p, self.q)


def validate_poses(poses):
    previous = None
    for pose in poses:
        if type(pose.t) is not int or pose.t < 0 or (previous is not None and pose.t <= previous):
            raise ValueError('trajectory timestamps must be strictly increasing integers')
        pose.matrix()
        previous = pose.t


def ground_truth(data):
    # CSV holds T_R_S; YAML holds T_B_S, and imu0 YAML holds T_B_I.
    sensor_T_imu = np.linalg.inv(data.gt_sensor['transform']) @ data.imu.calibration['transform']
    poses = []
    for t, row in zip(data.gt_times, data.gt_values):
        world_T_imu = pose_matrix(row[:3], row[[4, 5, 6, 3]]) @ sensor_T_imu
        poses.append(Pose(t, world_T_imu[:3, 3], quaternion(world_T_imu[:3, :3])))
    return poses


def associate(reference, estimate, tolerance_ns):
    """Greedy smallest residual, unique monotonic pairs; deterministic index ties."""
    if tolerance_ns < 0:
        raise ValueError('association tolerance must be nonnegative')
    validate_poses(reference)
    validate_poses(estimate)
    ref_times = [p.t for p in reference]
    candidates = []
    for j, pose in enumerate(estimate):
        start = bisect_left(ref_times, pose.t - tolerance_ns)
        for i in range(start, len(reference)):
            dt = reference[i].t - pose.t
            if dt > tolerance_ns:
                break
            candidates.append((abs(dt), i, j))
    used_ref, used_est, pairs, ordered_ref = set(), set(), [], []
    for _, i, j in sorted(candidates):
        if i not in used_ref and j not in used_est:
            k = bisect_left(ordered_ref, i)
            if (k > 0 and pairs[k-1][1] >= j) or (k < len(pairs) and pairs[k][1] <= j):
                continue
            pairs.insert(k, (i, j))
            ordered_ref.insert(k, i)
            used_ref.add(i)
            used_est.add(j)
    return pairs


def alignment(reference, estimate):
    g, e = np.array([p.p for p in reference]), np.array([p.p for p in estimate])
    gc, ec = g.mean(axis=0), e.mean(axis=0)
    covariance = (e - ec).T @ (g - gc)
    u, _, vt = np.linalg.svd(covariance)
    rank = int(np.linalg.matrix_rank(covariance))
    if rank == 0:
        # Constant positions determine no alignment rotation.
        r = reference[0].matrix()[:3, :3] @ estimate[0].matrix()[:3, :3].T
    else:
        correction = np.eye(3)
        correction[2, 2] = np.linalg.det(vt.T @ u.T)
        r = vt.T @ correction @ u.T
        if rank == 1:
            # The dominant singular vectors determine a directed line match.
            # Preserve that constraint, choosing only its free spin from pose orientation.
            axis = vt[0]
            x, y, z = axis
            skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            orientation = reference[0].matrix()[:3, :3] @ estimate[0].matrix()[:3, :3].T
            relative = orientation @ r.T
            angle = np.arctan2(np.sum(skew * relative), np.trace(relative) - axis @ relative @ axis)
            spin = np.eye(3) + np.sin(angle) * skew + (1. - np.cos(angle)) * (skew @ skew)
            r = spin @ r
    return r, gc - r @ ec, rank


def evaluate(gt, estimates, expected_times, tolerance_ns=10_000_000, interval_s=1.):
    if not np.isfinite(interval_s) or interval_s <= 0:
        raise ValueError('RPE interval must be positive and finite')
    validate_poses(gt)
    validate_poses(estimates)
    if any(type(t) is not int for t in expected_times) or any(b <= a for a, b in zip(expected_times, expected_times[1:])):
        raise ValueError('expected timestamps must be strictly increasing integers')
    result = dict(schema_version=1, status='no_output' if not estimates else 'failed',
                  expected_frames=len(expected_times), estimate_samples=len(estimates), matched_samples=0,
                  coverage=0., association_tolerance_ns=tolerance_ns, alignment='SE(3), scale fixed at 1',
                  body_frame='imu0/base_footprint', rpe_interval_s=interval_s,
                  ate_translation_rmse_m=None, rpe_translation_rmse_m=None,
                  rpe_rotation_rmse_rad=None, rpe_pairs=0)
    pairs = associate(gt, estimates, tolerance_ns)
    result['matched_samples'] = len(pairs)
    expected = [Pose(t, np.zeros(3), np.array([0., 0., 0., 1.])) for t in expected_times]
    gt_supported = {i for i, _ in associate(expected, gt, tolerance_ns)}
    matched_est = {j for _, j in pairs}
    covered = {i for i, j in associate(expected, estimates, tolerance_ns) if j in matched_est}
    result.update(covered_frames=len(covered), gt_supported_frames=len(gt_supported),
                  coverage=len(covered) / len(expected_times) if expected_times else 0.,
                  first_estimate_ns=estimates[0].t if estimates else None,
                  last_estimate_ns=estimates[-1].t if estimates else None)
    if len(pairs) < 3:
        result['reason'] = 'need at least three unique GT associations for rigid alignment'
        return result
    refs, ests = [gt[i] for i, _ in pairs], [estimates[j] for _, j in pairs]
    r, translation, rank = alignment(refs, ests)
    errors = np.array([r @ e.p + translation - g.p for g, e in zip(refs, ests)])
    result.update(status='ok' if len(covered) == len(expected_times) else 'partial_output',
                  ate_translation_rmse_m=float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
                  ate_translation_median_m=float(np.median(np.linalg.norm(errors, axis=1))),
                  ate_translation_max_m=float(np.max(np.linalg.norm(errors, axis=1))),
                  alignment_rotation=r.tolist(), alignment_translation=translation.tolist(), alignment_position_rank=rank,
                  association_max_error_ns=max(abs(g.t - e.t) for g, e in zip(refs, ests)))
    stamps = [e.t for e in ests]
    interval_ns = round(interval_s * 1_000_000_000)
    trans_errors, rot_errors = [], []
    for i, est in enumerate(ests):
        target = est.t + interval_ns
        k = bisect_left(stamps, target)
        candidates = [j for j in (k-1, k) if i < j < len(stamps)]
        if not candidates:
            continue
        j = min(candidates, key=lambda x: abs(stamps[x] - target))
        if abs(stamps[j] - target) > tolerance_ns:
            continue
        dg = np.linalg.inv(refs[i].matrix()) @ refs[j].matrix()
        de = np.linalg.inv(est.matrix()) @ ests[j].matrix()
        error = np.linalg.inv(dg) @ de
        trans_errors.append(np.linalg.norm(error[:3, 3]))
        rot_errors.append(np.arccos(np.clip((np.trace(error[:3, :3])-1)/2, -1., 1.)))
    if trans_errors:
        result.update(rpe_translation_rmse_m=float(np.sqrt(np.mean(np.square(trans_errors)))),
                      rpe_rotation_rmse_rad=float(np.sqrt(np.mean(np.square(rot_errors)))),
                      rpe_pairs=len(trans_errors))
    else:
        result['rpe_reason'] = 'no pose pairs within the requested fixed time interval tolerance'
    return result


def read_trajectory(path):
    poses = []
    with Path(path).open(encoding='utf-8', newline='') as f:
        for row in csv.DictReader(f):
            poses.append(Pose(timestamp(row['timestamp_ns']),
                              np.array([float(row[k]) for k in ('px', 'py', 'pz')]),
                              np.array([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])))
    validate_poses(poses)
    return poses


def save_evaluation(data, estimates, output, tolerance_ns=10_000_000, interval_s=1., failure=None):
    output = Path(output)
    gt = ground_truth(data)
    score = evaluate(gt, estimates, data.cameras[0].times, tolerance_ns, interval_s)
    score['reference_interpretation'] = ('EuRoC supplied reference; for MH sequences orientation is observed only by IMU '
                                         'in the Leica+IMU batch solution, not independent motion-capture orientation')
    if failure:
        score.update(status='failed', runtime_failure=failure)
    with (output / 'metrics.json').open('x', encoding='utf-8') as f:
        json.dump(score, f, indent=2, allow_nan=False)
        f.write('\n')
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection='3d')
    points = np.array([g.p for g in gt])
    ax.plot(*points.T, label='EuRoC GT (IMU origin)', linewidth=1)
    if estimates:
        points = np.array([e.p for e in estimates])
        if 'alignment_rotation' in score:
            points = points @ np.array(score['alignment_rotation']).T + score['alignment_translation']
        ax.plot(*points.T, label='estimate (SE(3) aligned)' if 'alignment_rotation' in score else 'unaligned estimate')
    ax.set(xlabel='x [m]', ylabel='y [m]', zlabel='z [m]', title=f"EuRoC: {score['status']}, coverage={score['coverage']:.1%}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / 'trajectory.png', dpi=180)
    plt.close(fig)
    return score


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--trajectory', required=True, help='CSV: timestamp_ns,px,py,pz,qx,qy,qz,qw (IMU/body poses)')
    parser.add_argument('--output-dir', required=True, help='New directory; never overwrites a previous evaluation')
    parser.add_argument('--tolerance-ns', type=int, default=10_000_000)
    parser.add_argument('--interval-s', type=float, default=1.)
    args = parser.parse_args(argv)
    try:
        data = load_dataset(args.dataset)
        estimates = read_trajectory(args.trajectory)
        output = Path(args.output_dir).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=False)
        score = save_evaluation(data, estimates, output, args.tolerance_ns, args.interval_s)
        print(json.dumps(score, indent=2))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'trajectory_evaluate: {exc}\n')
    return 0 if score['status'] in ('ok', 'partial_output') else 1


if __name__ == '__main__':
    raise SystemExit(main())
