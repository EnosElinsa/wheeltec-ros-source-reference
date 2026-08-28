# Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Check that source files carry copyright notices."""
    assert main(argv=[".", "test"]) == 0
