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

"""Loader and validator for the committed gate config (``interface_gates.yaml``).

The config maps each gate to a :class:`~autoware_interface_spec_lint.severity.Severity`
and carries the parameters the manifest audits need (the ``no_raw_spec_topic``
deny/allow lists and the ``owner_isolation`` base owner). Unknown gate names are
rejected, and a gate that is not implemented in this package may only be declared
``off`` (the honest-deferral contract).
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from pathlib import Path

from autoware_interface_spec_lint.severity import Severity
import yaml

# Gates that have a runnable implementation in this package.
IMPLEMENTED_GATES = frozenset(
    {
        "interface_spec_concept",
        "spec_registered",
        "version_consistency",
        "qos_consistency",
        "manifest_fresh",
        "owner_isolation",
        "no_raw_spec_topic",
    }
)

# Gates that are recognized but have no implementation in this package yet.
# They may appear in the config only at severity ``off``; enabling one is rejected
# because there is no implementation to run yet.
DEFERRED_GATES = frozenset(
    {
        "no_foreign_if_dependency",  # needs per-component required-role manifests
        "profile_compat",  # needs a vendor-specific interface profile that does not exist yet
        "if_usage_coverage",  # runtime gate; needs runtime introspection data this package does not have
        "admission_smoke",  # runtime gate; needs runtime introspection data this package does not have
    }
)

KNOWN_GATES = IMPLEMENTED_GATES | DEFERRED_GATES


@dataclass
class GateConfig:
    """Parsed, validated gate config."""

    severities: dict = field(default_factory=dict)  # gate name -> Severity
    no_raw_spec_topic_deny: list = field(default_factory=list)
    no_raw_spec_topic_allow: list = field(default_factory=list)
    owner_isolation_base_owner: str = "autowarefoundation"

    def severity(self, gate: str) -> Severity:
        return self.severities.get(gate, Severity.OFF)


def _parse_severity(gate: str, raw) -> Severity:
    if isinstance(raw, dict):
        raw = raw.get("severity")
    # YAML 1.1 parses the bare word `off` as boolean False; normalize it back so
    # `severity: off` reads naturally (a bare word `on`/True is left invalid).
    if raw is False:
        raw = "off"
    elif raw is True:
        raw = "on"
    if not isinstance(raw, str):
        raise ValueError(f"gate '{gate}': missing or non-string severity ({raw!r})")
    try:
        return Severity(raw.strip().lower())
    except ValueError as exc:
        valid = ", ".join(s.value for s in Severity)
        raise ValueError(f"gate '{gate}': unknown severity '{raw}' (valid: {valid})") from exc


def load_config(path) -> GateConfig:
    """Load and validate the gate config at ``path``."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    gates = data.get("gates") or {}
    if not isinstance(gates, dict):
        raise ValueError("gate config: 'gates' must be a mapping of gate -> {severity}")

    severities: dict = {}
    for gate, raw in gates.items():
        if gate not in KNOWN_GATES:
            valid = ", ".join(sorted(KNOWN_GATES))
            raise ValueError(f"gate config: unknown gate '{gate}' (known gates: {valid})")
        severity = _parse_severity(gate, raw)
        if severity is not Severity.OFF and gate not in IMPLEMENTED_GATES:
            raise ValueError(
                f"gate config: gate '{gate}' is deferred and has no implementation; "
                f"it may only be declared 'off'"
            )
        severities[gate] = severity

    raw_topic_cfg = data.get("no_raw_spec_topic") or {}
    owner_iso = data.get("owner_isolation") or {}
    return GateConfig(
        severities=severities,
        no_raw_spec_topic_deny=list(raw_topic_cfg.get("deny", [])),
        no_raw_spec_topic_allow=list(raw_topic_cfg.get("allow", [])),
        owner_isolation_base_owner=owner_iso.get("base_owner", "autowarefoundation"),
    )
