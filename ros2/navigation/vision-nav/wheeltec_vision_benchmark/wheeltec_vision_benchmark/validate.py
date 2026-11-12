"""Validate EuRoC raw stereo/IMU/GT, decode images and optionally create fixed VINS configs."""
import argparse
import json
from pathlib import Path
from .config import generate_config
from .dataset import load_dataset
from .rectification import StereoRectifier


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, help='Sequence directory or mav0')
    parser.add_argument('--output', help='New directory for dataset identity and generated configs')
    parser.add_argument('--mode', choices=('stereo', 'vio'), default='vio')
    args = parser.parse_args(argv)
    try:
        data = load_dataset(args.dataset)
        rect = StereoRectifier(data)
        for side, camera in enumerate(data.cameras):
            for path in camera.paths:
                rect.image(path, side)
        report = dict(schema_version=1, valid=True, data=data.identity(),
                      rectified_camera_info=[rect.camera_info(i) for i in range(2)],
                      body_T_rect=[t.tolist() for t in rect.body_T_rect])
        report_path = None
        if args.output:
            out = Path(args.output).expanduser().resolve()
            out.mkdir(parents=True, exist_ok=False)
            generate_config(data, rect, out / 'config', args.mode)
            report_path = out / 'validation.json'
            report_path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        summary = dict(schema_version=1, valid=True,
                       data={key: report['data'][key] for key in ('mav0', 'sha256', 'camera_frames', 'imu_samples', 'gt_samples')},
                       file_count=len(report['data']['files']),
                       validation_json=str(report_path) if report_path else None,
                       identity_note='Full per-file hashes are saved with --output; stdout contains the aggregate identity.')
        print(json.dumps(summary, indent=2))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f'validate_euroc: {exc}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
