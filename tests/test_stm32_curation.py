from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_semantic_stm32_projects_have_main_sources():
    paths = (
        "stm32/labs/uart/stm32f103-stdperiph/main.c",
        "stm32/labs/pwm/stm32f103-stdperiph/main.c",
        "stm32/labs/encoder/stm32f103-stdperiph/main.c",
        "stm32/control/motor/stm32f4-stdperiph/main.c",
        "stm32/control/pid/stm32f4-stdperiph/main.c",
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
