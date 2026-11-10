"""Validated shared robot, perception, navigation and safety configuration."""
from dataclasses import asdict, fields
from pathlib import Path
import math
import re

import yaml

from wheeltec_vision_nav.runtime_safety import SafetyConfig


# The installed supervised trees implement this fixed, bounded recovery policy.
# Reject different budgets before leases, generated files or DB operations; do
# not let a configured guard disagree with the requested BT recovery action.
SUPPORTED_REVERSE_LIMITS = {"reverse_speed": 0.08, "reverse_m": 0.25, "reverse_s": 3.2}


def load_runtime_config(path):
    cfg = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or set(cfg) != {"robot", "perception", "navigation", "safety"}:
        raise ValueError("runtime.yaml requires robot, perception, navigation and safety sections")
    if any(not isinstance(value, dict) for value in cfg.values()):
        raise ValueError("runtime sections must be mappings")
    validate_runtime_config(cfg)
    return cfg


def validate_runtime_config(cfg):
    if cfg["robot"].get("car_mode") != "senior_akm":
        raise ValueError("Only the fixed senior_akm hardware is supported")
    profile = cfg["perception"].get("camera_profile", "")
    if not re.fullmatch(r"[1-9]\d*x[1-9]\d*x[1-9]\d*", str(profile)):
        raise ValueError("camera_profile must be WIDTHxHEIGHTxFPS")
    for section, name in (("perception", "max_features"), ("perception", "cloud_decimation"), ("navigation", "mppi_batch_size")):
        value = cfg[section].get(name)
        if type(value) is not int or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    for name in ("odom_tf_linear_variance", "odom_tf_angular_variance"):
        value = cfg["perception"].get(name)
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    safety_keys = {item.name for item in fields(SafetyConfig)} - {"mode", "wheel_backup"}
    if set(cfg["safety"]) != safety_keys:
        raise ValueError(f"safety must provide all SafetyConfig fields except mode/wheel_backup; missing={safety_keys - set(cfg['safety'])}, unknown={set(cfg['safety']) - safety_keys}")
    defaults = asdict(SafetyConfig())
    for name, value in cfg["safety"].items():
        expected = type(defaults[name])
        valid = (type(value) is expected if expected in (bool, int)
                 else type(value) in (float, int))
        if not valid:
            raise ValueError(f"safety.{name} must have type {expected.__name__}")
    if cfg["safety"].get("require_depth") is not True:
        raise ValueError("Raw depth, cloud and obstacle evidence is required")
    SafetyConfig(**cfg["safety"])
    for name, supported in SUPPORTED_REVERSE_LIMITS.items():
        if cfg["safety"][name] != supported:
            raise ValueError(f"safety.{name} must remain {supported}: the installed supervised recovery trees support only the fixed reverse policy")
    return cfg


def resolve_odom_source(odom_source="auto", *, start_base=False, hold_odom=False):
    if odom_source not in ("auto", "ekf", "vo_only", "external"):
        raise ValueError("odom_source must be auto, ekf, vo_only or external")
    if hold_odom:
        if odom_source not in ("auto", "vo_only"):
            raise ValueError("hold_odom requires vo_only; it conflicts with EKF/external ownership")
        return "vo_only"
    return ("ekf" if start_base else "vo_only") if odom_source == "auto" else odom_source


def safety_parameters(cfg, *, localization, odom_source, supervised_reverse=None):
    params = dict(cfg["safety"])
    params.update(mode="navigation" if localization else "mapping", wheel_backup=odom_source == "ekf")
    if supervised_reverse is not None:
        params["supervised_reverse"] = supervised_reverse
    return asdict(SafetyConfig(**params))
