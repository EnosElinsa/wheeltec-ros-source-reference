"""Camera extrinsics must be finite, complete, and preserve omitted orientation."""
from pathlib import Path

import pytest

from wheeltec_vision_nav.camera_model import parse_rpy, parse_xyz, rewrite_camera_joint, write_camera_urdf


URDF = ('<robot><joint type="fixed" name="camera_joint">'
        '<parent link="base_link"/><origin rpy="0.1 0.2 0.3" xyz="0 0 0"/>'
        '<child link="camera_link"/></joint></robot>')


@pytest.mark.parametrize("text", ["nan 0 0", "0 inf 0", "0 0 -inf", "1 2", "1 2 3 4"])
def test_coordinates_and_angles_reject_nonfinite_or_incomplete_values(text):
    with pytest.raises(ValueError):
        parse_xyz(text)
    with pytest.raises(ValueError):
        parse_rpy(text)


def test_optional_angles_preserve_authored_orientation_and_accept_full_radians():
    assert parse_rpy("") is None
    assert parse_rpy(None) is None
    assert parse_rpy("0.01, -0.02, 0.03") == (.01, -.02, .03)
    assert 'rpy="0.1 0.2 0.3"' in rewrite_camera_joint(URDF, (.1, .2, .3))
    updated = rewrite_camera_joint(URDF, (.1, .2, .3), (.4, -.5, .6))
    assert 'rpy="0.400000 -0.500000 0.600000"' in updated
    assert 'xyz="0.100000 0.200000 0.300000"' in updated
    assert '<parent link="base_link"/>' in updated


def test_complete_orientation_can_be_added_to_origin_without_rpy():
    source = '<robot><joint name="camera_joint"><origin xyz="0 0 0"/></joint></robot>'
    updated = rewrite_camera_joint(source, (1., 2., 3.), "0 0 1")
    assert 'rpy="0.000000 0.000000 1.000000"' in updated


def test_file_writer_converts_footprint_xyz_and_preserves_explicit_orientation(tmp_path):
    source = tmp_path / "robot.urdf"
    source.write_text(URDF, encoding="utf-8")
    generated = Path(write_camera_urdf(str(source), (.2, .02, .13), .04, (.1, .2, .3)))
    try:
        text = generated.read_text(encoding="utf-8")
        assert 'xyz="0.200000 0.020000 0.090000"' in text
        assert 'rpy="0.100000 0.200000 0.300000"' in text
        assert source.read_text(encoding="utf-8") == URDF
    finally:
        generated.unlink()
    with pytest.raises(ValueError):
        write_camera_urdf(str(source), (.2, .02, .13), float("nan"))


def test_programmatic_nonfinite_extrinsics_also_fail():
    with pytest.raises(ValueError):
        rewrite_camera_joint(URDF, (float("nan"), 0., 0.))
    with pytest.raises(ValueError):
        rewrite_camera_joint(URDF, (0., 0., 0.), (0., float("inf"), 0.))
