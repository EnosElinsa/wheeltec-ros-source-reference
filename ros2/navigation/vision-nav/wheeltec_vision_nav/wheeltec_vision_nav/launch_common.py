"""Installed resource paths and topic names shared by the visual launch modules."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from wheeltec_vision_nav.nav_params import OBSTACLES_TOPIC


def package_file(package: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package), *parts))


def launch_flag(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def rtabmap_visual_config() -> str:
    return package_file("wheeltec_vision_nav", "config", "rtabmap_visual.yaml")


def ekf_visual_config() -> str:
    return package_file("wheeltec_vision_nav", "config", "ekf_visual.yaml")


def ackermann_bt_xml(supervised_reverse=False, through_poses=False) -> str:
    stem = "navigate_through_poses_ackermann" if through_poses else "navigate_w_replanning_ackermann"
    if not supervised_reverse:
        stem += "_forward"
    return package_file(
        "wheeltec_vision_nav",
        "behavior_trees",
        stem + ".xml",
    )


PAIRED_DEPTH_TOPIC = "/vision/paired/depth/image_rect_raw"
PAIRED_DEPTH_INFO_TOPIC = "/vision/paired/depth/camera_info"
DEPTH_CLOUD_TOPIC = "/vision/depth_cloud"
RAW_DEPTH_TOPIC = "/camera/depth/image_rect_raw"
RAW_DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
