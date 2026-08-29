from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_selected_ros2_packages_have_complete_entrypoints():
    required = {
        "ros2/robot/turn-on-wheeltec-robot/package.xml",
        "ros2/navigation/nav2-waypoint-cycle/nav2_waypoint_cycle/package.xml",
        "ros2/navigation/path-follow/wheeltec_path_follow/package.xml",
        "applications/kcf-tracker/wheeltec_robot_kcf_model/package.xml",
        "examples/ros2/pubsub/cpp/package.xml",
        "examples/ros2/pubsub/python/package.xml",
    }
    assert all((ROOT / item).is_file() for item in required)
