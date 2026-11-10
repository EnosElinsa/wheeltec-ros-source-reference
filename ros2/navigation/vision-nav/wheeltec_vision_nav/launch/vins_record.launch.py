"""Record already-running sensors. Never starts drivers or moves the robot."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from wheeltec_vision_nav.vins_experiment import RECORD_TOPICS


def generate_launch_description():
    def build(context):
        dest = Path(LaunchConfiguration("output").perform(context)).expanduser()
        if dest.exists():
            raise ValueError("recording destination already exists")
        qos = Path(get_package_share_directory("wheeltec_vision_nav")) / "config/vins_qos.yaml"
        return [ExecuteProcess(cmd=["ros2", "bag", "record", "-o", str(dest),
                    "--qos-profile-overrides-path", str(qos), *RECORD_TOPICS], output="screen")]
    return LaunchDescription([DeclareLaunchArgument("output", description="New raw sensor bag directory"),
                              OpaqueFunction(function=build)])
