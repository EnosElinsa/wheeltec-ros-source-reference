import json
import pytest
from wheeltec_vision_nav.vins_config import make_config
from wheeltec_vision_nav.vins_audit import load_calibration


def test_generate_rectified_intrinsics_from_audited_camera_info(tmp_path):
    p = [420., 0., 420., 0., 0., 420., 240., 0., 0., 0., 1., 0.]
    model = {"width": 848, "height": 480, "p": p, "frame_id": "left"}
    right = {**model, "p": [*p[:3], -21., *p[4:]], "frame_id": "right"}
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"stereo_ready": True, "camera_models": {"left": model, "right": right}}))
    out = tmp_path / "generated"
    path = make_config(audit, out, "stereo")
    cfg = load_calibration(path)
    cam = load_calibration(out / cfg["cam1_calib"])
    assert cfg["imu"] == 0 and cfg["estimate_extrinsic"] == 0
    assert cam["projection_parameters"] == {"fx": 420., "fy": 420., "cx": 420., "cy": 240.}
    assert cam["distortion_parameters"] == {"k1": 0., "k2": 0., "p1": 0., "p2": 0.}
    assert cam["image_width"] == 848
    with pytest.raises(FileExistsError):
        make_config(audit, out, "stereo")


def test_vio_requires_real_calibration_no_d435i_or_identity_defaults(tmp_path):
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"stereo_ready": True}))
    with pytest.raises(ValueError, match="calibration"):
        make_config(audit, tmp_path / "out", "vio")


def test_generated_yaml_is_readable_by_real_opencv(tmp_path):
    cv2 = pytest.importorskip("cv2")
    test_generate_rectified_intrinsics_from_audited_camera_info(tmp_path)
    fs = cv2.FileStorage(str(tmp_path / "generated/vins.yaml"), cv2.FILE_STORAGE_READ)
    try:
        assert fs.isOpened()
        assert fs.getNode("body_T_cam0").mat().shape == (4, 4)
        assert fs.getNode("body_T_cam1").mat()[0, 3] == pytest.approx(.05)
        assert fs.getNode("num_of_cam").real() == 2
    finally:
        fs.release()
    camera = cv2.FileStorage(str(tmp_path / "generated/left.yaml"), cv2.FILE_STORAGE_READ)
    try:
        assert camera.getNode("projection_parameters").getNode("fx").real() == 420.
    finally:
        camera.release()


def test_vio_rejects_measured_time_offset_unsupported_by_pinned_wrapper(tmp_path):
    test_generate_rectified_intrinsics_from_audited_camera_info(tmp_path)
    measured = tmp_path / "measured.yaml"
    cfg = load_calibration(tmp_path / "generated/vins.yaml")
    cfg.update(imu=1, td=.1)
    measured.write_text(json.dumps(cfg))
    with pytest.raises(ValueError, match="td|time offset"):
        make_config(tmp_path / "audit.json", tmp_path / "vio", "vio", measured)
