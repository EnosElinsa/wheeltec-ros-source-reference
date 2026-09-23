"""Capture PNG previews and a YAML summary. Does not start mapping, navigation, or the chassis."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_utils import capture_runtime_node


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")

    def build(context, *args, **kwargs):
        return [capture_runtime_node(output_dir.perform(context))]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=os.path.expanduser("~/vision_captures"),
            ),
            OpaqueFunction(function=build),
        ]
    )
