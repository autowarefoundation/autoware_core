#!/usr/bin/env python3
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

"""Repo-local launcher for the pre-commit hook.

Runs the interface spec lint without requiring the package to be installed, so
the pre-commit hook works from a plain checkout (no colcon build, no pip). It
adds the package root to ``sys.path`` and delegates to the console entry's
``main``; the working directory (the repo root) is where the default spec dir
and manifest are resolved from.
"""

import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from autoware_interface_spec_lint.main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
