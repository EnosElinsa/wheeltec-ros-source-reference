from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_one_semantic_bodyreader_and_no_numbered_qt_roots():
    assert (ROOT / "ros1/maintenance/bodyreader").is_dir()
    assert not list(ROOT.rglob("qt-ros-test"))
