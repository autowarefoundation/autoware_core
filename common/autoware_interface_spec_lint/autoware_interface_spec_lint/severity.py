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

"""Gate severity and the severity-to-exit-code mapping (the warn->error ratchet).

A gate declared ``off`` is not run; a gate at ``warn`` is advisory (prints
findings, never fails); a gate at ``error`` fails the build when it has at least
one finding. The committed gate config can ratchet any gate from ``warn`` to
``error`` independently as its check is trusted.
"""

from __future__ import annotations

import enum


class Severity(enum.Enum):
    """Per-gate severity as declared in the committed gate config."""

    OFF = "off"
    WARN = "warn"
    ERROR = "error"


def exit_code_for(findings, severity: Severity) -> int:
    """Return 1 (fail) only at ERROR severity with at least one finding, else 0."""
    return 1 if severity is Severity.ERROR and findings else 0
