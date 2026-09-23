"""Record a baseline rosbag. Does not start lidar, chassis, or mapping."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from wheeltec_vision_nav.baseline_criteria import record_topics


def generate_launch_description():
    session = LaunchConfiguration("session")
    output = LaunchConfiguration("output")
    qos = str(Path(get_package_share_directory("wheeltec_vision_nav"), "config", "baseline_qos_overrides.yaml"))

    def build(context, *args, **kwargs):
        name = session.perform(context)
        dest = output.perform(context)
        topics = list(record_topics(name))
        cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            dest,
            "--qos-profile-overrides-path",
            qos,
            *topics,
        ]
        return [ExecuteProcess(cmd=cmd, output="screen")]

    return LaunchDescription(
        [
            DeclareLaunchArgument("session", default_value="vision_base"),
            DeclareLaunchArgument("output", default_value="/home/wheeltec/bags/r680_baseline"),
            OpaqueFunction(function=build),
        ]
    )
