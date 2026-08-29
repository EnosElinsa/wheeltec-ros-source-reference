from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_semantic_stm32_projects_have_main_sources():
    paths = (
        "stm32/labs/uart/stm32f103-stdperiph/USER/main.c",
        "stm32/labs/pwm/stm32f103-stdperiph/USER/main.c",
        "stm32/labs/encoder/stm32f103-stdperiph/USER/main.c",
        "stm32/control/motor/stm32f4-stdperiph/USER/main.c",
        "stm32/control/pid/stm32f4-stdperiph/USER/main.c",
    )
    assert all((ROOT / path).is_file() for path in paths)

def test_projects_keep_uv_files_without_duplicate_root_main_copies():
    roots = (
        "stm32/labs/uart/stm32f103-stdperiph",
        "stm32/labs/pwm/stm32f103-stdperiph",
        "stm32/labs/encoder/stm32f103-stdperiph",
        "stm32/control/motor/stm32f4-stdperiph",
        "stm32/control/pid/stm32f4-stdperiph",
    )
    for root in roots:
        assert list((ROOT / root).rglob("*.uvprojx"))
        assert not (ROOT / root / "main.c").exists()

def test_uv_projects_have_resolved_file_paths_and_include_headers():
    for root in ("stm32/labs/uart/stm32f103-stdperiph", "stm32/labs/pwm/stm32f103-stdperiph", "stm32/labs/encoder/stm32f103-stdperiph", "stm32/control/motor/stm32f4-stdperiph", "stm32/control/pid/stm32f4-stdperiph"):
        uv = next((ROOT / root).rglob("*.uvprojx"))
        files = [n.text for n in ET.parse(uv).findall('.//FilePath') if n.text]
        assert files
        assert all((uv.parent / Path(item.replace('\\\\','/'))).is_file() for item in files)

def test_no_generated_outputs_or_backup_files_in_curated_roots():
    for root in (ROOT / "stm32/labs", ROOT / "stm32/control"):
        assert not [p for p in root.rglob("*") if p.is_file() and (p.suffix.lower() in {".hex", ".bin", ".axf", ".lst", ".o", ".obj"} or "backup" in p.name.lower())]
