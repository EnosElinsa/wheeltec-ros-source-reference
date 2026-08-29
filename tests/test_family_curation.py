from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent))
from family_curation import classify_family_variants, validate_no_upstream_mirror

ROOT = Path(__file__).resolve().parents[1]


def test_one_semantic_bodyreader_and_no_numbered_qt_roots():
    assert (ROOT / "ros1/maintenance/bodyreader").is_dir()
    assert not list(ROOT.rglob("qt-ros-test"))

def test_bodyreader_is_complete_and_curated_scopes_have_no_numbered_roots():
    body=ROOT/'ros1/maintenance/bodyreader'
    for item in ('package.xml','CMakeLists.txt','launch/bodyfollow.launch','msg/body.msg','src/bodydata_process.cpp'):
        assert (body/item).is_file()
    assert classify_family_variants([body])=={'bodyreader':1}
    for scope in ('ros1','ros2','applications','platform'):
        assert not [item for item in (ROOT/scope).glob('*') if item.name[:1].isdigit()]
    assert validate_no_upstream_mirror(body)==[]
    package = (body / "package.xml").read_text(encoding="utf-8")
    assert "<license>user-confirmed-public</license>" in package
    assert "<build_depend>geometry_msgs</build_depend>" in package
    assert "<exec_depend>sensor_msgs</exec_depend>" in package
