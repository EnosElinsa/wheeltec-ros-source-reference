# Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check Python formatting."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, "\n".join(errors)
