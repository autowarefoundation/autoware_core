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

from autoware_interface_spec_lint.severity import Severity
from autoware_interface_spec_lint.severity import exit_code_for


def test_warn_never_fails_even_with_findings():
    assert exit_code_for(["divergent: /planning/trajectory"], Severity.WARN) == 0


def test_error_fails_when_findings_present():
    assert exit_code_for(["divergent: /planning/trajectory"], Severity.ERROR) == 1


def test_error_passes_when_clean():
    assert exit_code_for([], Severity.ERROR) == 0


def test_off_never_fails():
    assert exit_code_for(["anything"], Severity.OFF) == 0
