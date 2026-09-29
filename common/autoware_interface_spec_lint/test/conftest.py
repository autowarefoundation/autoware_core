# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Make the real manifest generator discoverable to the manifest_fresh test in CI.

`manifest_fresh` needs the built ``generate_interface_manifest`` binary and otherwise
skips. Locally a developer exports ``INTERFACE_MANIFEST_GENERATOR``; in CI nothing does,
so the freshness test used to skip on the runner even though the same drift is already a
hard failure in the specs package's own C++ test. This locates the binary through the
ament index -- the specs package installs it under ``<prefix>/lib/<pkg>/`` -- and exports
the variable before the tests run, so the colcon test exercises the real generator too.

The ``autoware_component_interface_specs`` test dependency in package.xml is what makes
that install present in this package's colcon build+test. A pre-set variable always wins,
so a developer can still point the test at a hand-built binary.
"""

from __future__ import annotations

import os
from pathlib import Path

from autoware_interface_spec_lint.checks import GENERATOR_ENV

_GENERATOR_PACKAGE = "autoware_component_interface_specs"
_GENERATOR_NAME = "generate_interface_manifest"


def _discover_generator() -> Path | None:
    try:
        from ament_index_python.packages import get_package_prefix
    except ImportError:
        return None  # ament not on the path (e.g. a bare `pytest` run); leave it to skip.
    try:
        prefix = Path(get_package_prefix(_GENERATOR_PACKAGE))
    except Exception:
        return None  # specs package not installed in this workspace.
    candidate = prefix / "lib" / _GENERATOR_PACKAGE / _GENERATOR_NAME
    return candidate if candidate.is_file() else None


def pytest_configure(config):
    if os.environ.get(GENERATOR_ENV):
        return  # An explicit path (local dev) always wins over discovery.
    generator = _discover_generator()
    if generator is not None:
        os.environ[GENERATOR_ENV] = str(generator)
