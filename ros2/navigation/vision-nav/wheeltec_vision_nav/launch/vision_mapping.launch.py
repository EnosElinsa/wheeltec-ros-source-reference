"""Leased visual mapping for the fixed senior_akm robot."""
from wheeltec_vision_nav.launch_pipeline import generate_visual_launch_description


def generate_launch_description():
    return generate_visual_launch_description(localization=False)
