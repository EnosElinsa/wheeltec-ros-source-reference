"""Camera coordinates and URDF rewriting, independent of ROS launch."""

import os
import math
import re
import tempfile
from pathlib import Path

# D435C left-infrared optical centre in base_footprint (m). Derived from the
# sensor-mount-v1 CAD: top plate upper face ~0.104 m above base_footprint,
# bracket M3 holes 28 mm above the plate, camera front face 9.5 mm behind the
# plate front edge (~0.197 m), left imager 17.5 mm left of the centreline.
# The RealSense driver roots its TF tree at camera_link (= depth/infra1
# sensor origin), so this is the pose that base_link -> camera_link must give.
DEFAULT_CAMERA_XYZ = (0.187, 0.0175, 0.132)


def parse_xyz(value, default=DEFAULT_CAMERA_XYZ) -> tuple[float, float, float]:
    """Parse 'x y z' or 'x,y,z' (metres). Empty string -> default."""
    text = str(value or "").strip()
    if not text:
        return _finite_triple(default, "camera_xyz")
    parts = [p for p in re.split(r"[\s,]+", text) if p]
    if len(parts) != 3:
        raise ValueError(f"camera_xyz needs three numbers, got {value!r}")
    return _finite_triple(parts, "camera_xyz")


def _finite_triple(values, name):
    try:
        result = tuple(float(v) for v in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} needs three finite numbers") from exc
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} needs three finite numbers")
    return result


def parse_rpy(value) -> tuple[float, float, float] | None:
    """Parse complete roll/pitch/yaw radians; empty preserves the authored URDF pose."""
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    parts = re.split(r"[\s,]+", value.strip()) if isinstance(value, str) else value
    return _finite_triple(parts, "camera_rpy")


def rewrite_camera_joint(urdf_text: str, xyz_in_base_link: tuple[float, float, float],
                         camera_rpy=None) -> str:
    """Move the URDF camera_joint origin so camera_link sits at the D435C.

    The factory senior_akm URDF places camera_link at the former Orbbec camera.
    Optional complete camera_rpy replaces orientation; None preserves it.
    """
    xyz = _finite_triple(xyz_in_base_link, "camera_xyz")
    rpy = parse_rpy(camera_rpy)
    pattern = re.compile(
        r'''<joint\b(?=[^>]*\bname\s*=\s*["']camera_joint["'])[^>]*>.*?</joint>''', re.DOTALL)

    def rewrite_joint(match):
        def rewrite_origin(origin):
            text = origin.group(0)
            for attr, values in (("xyz", xyz), ("rpy", rpy)):
                if values is None:
                    continue
                value = " ".join(f"{v:.6f}" for v in values)
                attribute = re.compile(rf'''\b{attr}\s*=\s*(["']).*?\1''')
                replacement = f'{attr}="{value}"'
                if attribute.search(text):
                    text = attribute.sub(lambda _: replacement, text, count=1)
                else:
                    text = re.sub(r"\s*/?>$", lambda m: " " + replacement + m.group(0), text)
            return text
        text, count = re.subn(r"<origin\b[^>]*>", rewrite_origin, match.group(0), count=1)
        if count != 1:
            raise RuntimeError("camera_joint <origin> not found in URDF")
        return text

    text, count = pattern.subn(rewrite_joint, urdf_text, count=1)
    if count != 1:
        raise RuntimeError("camera_joint <origin xyz> not found in URDF")
    return text


def write_camera_urdf(urdf_path: str, camera_xyz_in_footprint, base_to_link_z: float,
                      camera_rpy=None) -> str:
    """Write a temp URDF whose camera_link is at camera_xyz (base_footprint frame)."""
    x, y, z = _finite_triple(camera_xyz_in_footprint, "camera_xyz")
    if not math.isfinite(base_to_link_z):
        raise ValueError("base_to_link_z must be finite")
    text = Path(urdf_path).read_text(encoding="utf-8")
    text = rewrite_camera_joint(text, (x, y, z - base_to_link_z), camera_rpy)
    handle, generated = tempfile.mkstemp(prefix="wheeltec_vision_nav_", suffix=".urdf")
    os.close(handle)
    Path(generated).write_text(text, encoding="utf-8")
    return generated
