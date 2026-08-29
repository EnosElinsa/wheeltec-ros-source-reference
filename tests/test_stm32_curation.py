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
