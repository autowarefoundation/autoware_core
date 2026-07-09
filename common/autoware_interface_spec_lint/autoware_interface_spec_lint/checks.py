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

"""Static and manifest checks for the Autoware component interface specs.

All checks are WARN-only in milestone M0: they report human-readable findings
but never fail the build. The warn->error ratchet is a later milestone (M2).
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

# Suppression marker (fixed string; other PRs depend on the exact text). A spec
# struct carrying this marker on its own line or on the line directly above its
# declaration is exempt from the spec_registered check.
SUPPRESS_MARKER = "// interface-spec-lint: not-versioned"

# Environment variable that points at the built M0.1 manifest generator binary.
GENERATOR_ENV = "INTERFACE_MANIFEST_GENERATOR"

# Headers that are not per-domain spec files and carry no domain version / Specs.
_SKIP_HEADERS = {"utils.hpp", "version.hpp", "concepts.hpp"}

# The header declaring the one QoS profile every service spec runs on.
_UTILS_HEADER = "utils.hpp"

_STRUCT_RE = re.compile(r"\bstruct\s+([A-Z]\w*)\s*\{(.*?)\}\s*;", re.DOTALL)
_SPECS_RE = re.compile(r"using\s+Specs\s*=\s*std::tuple<([^>]*)>\s*;")
_VERSION_RE = re.compile(r"constexpr\s+Version\s+version\s*\{([^}]*)\}\s*;")
_NAME_RE = re.compile(r"\bname\s*\[\s*\]")

# A domain may declare its version and its `Specs` tuple either literally or through
# AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(MAJOR, MINOR, PATCH, ...), which
# expands to both. clang-format wraps the invocation across lines, hence `[^)]*`.
_DEFINE_DOMAIN_RE = re.compile(r"\bAUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN\s*\(([^)]*)\)")

_NAME_VALUE_RE = re.compile(r'\bname\s*\[\s*\]\s*=\s*"([^"]*)"')
_DEPTH_VALUE_RE = re.compile(r"\bdepth\s*=\s*(\d+)\s*;")
_RELIABILITY_VALUE_RE = re.compile(r"\breliability\s*=\s*(\w+)\s*;")
_DURABILITY_VALUE_RE = re.compile(r"\bdurability\s*=\s*(\w+)\s*;")
_SERVICE_QOS_RE = re.compile(r"namespace\s+service_qos\s*\{([^}]*)\}")

# The RMW policy enumerators the manifest can name, and the spelling the generator's
# `to_string` emits for each. A policy outside these maps is rejected rather than
# guessed at, mirroring the generator: a spec that reaches for a policy the manifest
# cannot name must be an explicit decision, not a silently degraded entry.
_RELIABILITY_NAMES = {
    "RMW_QOS_POLICY_RELIABILITY_RELIABLE": "reliable",
    "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT": "best_effort",
}
_DURABILITY_NAMES = {
    "RMW_QOS_POLICY_DURABILITY_VOLATILE": "volatile",
    "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL": "transient_local",
}

# Both get_qos<Spec>() and get_service_qos() build an rclcpp::QoS from a depth alone,
# which is KEEP_LAST; no spec can currently ask for KEEP_ALL.
_HISTORY = "keep_last"


@dataclass
class Finding:
    """A single WARN-level diagnostic anchored at a file and line."""

    level: str  # "WARN" (M0); "ERROR" later.
    file: str
    line: int
    message: str


@dataclass
class Header:
    """Parsed view of one domain header used by every static check."""

    path: Path
    structs: dict = field(default_factory=dict)  # name -> info dict
    specs_members: list = field(default_factory=list)  # names in `using Specs`
    versions: list = field(default_factory=list)  # each `version{a,b,c}` body text
    version_lines: list = field(default_factory=list)  # 1-indexed lines of each version


def _line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _blank_comments(text: str) -> str:
    """Overwrite C/C++ comments with spaces, keeping every offset and newline.

    The scanners below anchor findings by byte offset and `_is_suppressed` reads the
    raw lines, so the blanked copy must stay the same length and keep its line breaks.
    Without this a declaration such as `struct Foo  // note` followed by `{` on the
    next line never matches `_STRUCT_RE`, and the struct becomes invisible to the
    static gates.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in ('"', "'"):  # skip a string/char literal; `//` inside it is not a comment
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                    continue
                if text[i] == c:
                    i += 1
                    break
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "*":
            out[i] = out[i + 1] = " "
            i += 2
            while i < n and not (text[i] == "*" and i + 1 < n and text[i + 1] == "/"):
                if text[i] != "\n":
                    out[i] = " "
                i += 1
            if i < n:
                out[i] = " "
                if i + 1 < n:
                    out[i + 1] = " "
                i += 2
            continue
        i += 1
    return "".join(out)


def _is_suppressed(lines: list, struct_line: int) -> bool:
    idx = struct_line - 1  # 0-indexed own line
    candidates = []
    if 0 <= idx < len(lines):
        candidates.append(lines[idx])
    if 0 <= idx - 1 < len(lines):
        candidates.append(lines[idx - 1])
    return any(SUPPRESS_MARKER in line for line in candidates)


def _group(match, default=None):
    return match.group(1) if match else default


def parse_header(path: Path) -> Header:
    """Extract structs, the Specs tuple members and version literals of a header."""
    text = path.read_text()
    lines = text.splitlines()  # raw: the suppression marker is itself a comment
    scan = _blank_comments(text)  # same offsets, comments neutralised
    structs: dict = {}
    for m in _STRUCT_RE.finditer(scan):
        name, body = m.group(1), m.group(2)
        struct_line = _line_of(scan, m.start())
        structs[name] = {
            "line": struct_line,
            "has_name": ("name[]" in body) or (_NAME_RE.search(body) is not None),
            "has_message": "using Message" in body,
            "has_service": "using Service" in body,
            "has_depth": "depth" in body,
            "has_reliability": "reliability" in body,
            "has_durability": "durability" in body,
            "suppressed": _is_suppressed(lines, struct_line),
            # Values, not just presence: qos_consistency compares them to the manifest.
            "interface": _group(_NAME_VALUE_RE.search(body)),
            "depth": _group(_DEPTH_VALUE_RE.search(body)),
            "reliability": _group(_RELIABILITY_VALUE_RE.search(body)),
            "durability": _group(_DURABILITY_VALUE_RE.search(body)),
        }
    specs_members: list = []
    versions = []
    version_lines = []
    sm = _SPECS_RE.search(scan)
    if sm:
        specs_members = [s.strip() for s in sm.group(1).split(",") if s.strip()]
    for vm in _VERSION_RE.finditer(scan):
        versions.append(vm.group(1).strip())
        version_lines.append(_line_of(scan, vm.start()))
    # The macro form expands to both a `version{...}` and a `using Specs = ...`, so a
    # domain that uses it declares exactly what the literal form above declares. Parsing
    # only the literal form would make every macro-using domain look version-less and
    # its every spec look unregistered.
    for dm in _DEFINE_DOMAIN_RE.finditer(scan):
        args = [a.strip() for a in dm.group(1).split(",") if a.strip()]
        if len(args) < 3:
            continue  # malformed; version_consistency reports the missing version
        versions.append(", ".join(args[:3]))
        version_lines.append(_line_of(scan, dm.start()))
        specs_members.extend(args[3:])
    return Header(
        path=path,
        structs=structs,
        specs_members=specs_members,
        versions=versions,
        version_lines=version_lines,
    )


def _domain_headers(spec_dir: Path) -> list:
    return sorted(p for p in Path(spec_dir).glob("*.hpp") if p.name not in _SKIP_HEADERS)


def _version_str(raw: str) -> str:
    return ".".join(part.strip() for part in raw.split(",") if part.strip())


def spec_registered(spec_dir: Path, _manifest=None) -> list:
    """WARN for each spec struct missing from its namespace's `using Specs` tuple.

    A struct carrying the SUPPRESS_MARKER on or directly above its declaration is
    treated as intentionally not-versioned and skipped.
    """
    findings: list = []
    for path in _domain_headers(spec_dir):
        h = parse_header(path)
        registered = set(h.specs_members)
        for name, info in h.structs.items():
            if not info["has_name"]:
                continue
            if info["suppressed"]:
                continue
            if name not in registered:
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        info["line"],
                        f"spec struct '{name}' is not registered in its namespace's "
                        f"'using Specs = std::tuple<...>'",
                    )
                )
    return findings


def interface_spec_concept(spec_dir: Path, _manifest=None) -> list:
    """WARN for a struct with a `name[]` that is neither a valid topic nor service.

    A valid topic has Message + depth + reliability + durability; a valid service
    has Service. This mirrors the compile-time AnySpec concept from M0.1.
    """
    findings: list = []
    for path in _domain_headers(spec_dir):
        h = parse_header(path)
        for name, info in h.structs.items():
            if not info["has_name"]:
                continue
            is_topic = (
                info["has_message"]
                and info["has_depth"]
                and info["has_reliability"]
                and info["has_durability"]
            )
            is_service = info["has_service"]
            if not (is_topic or is_service):
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        info["line"],
                        f"spec struct '{name}' satisfies neither the topic concept "
                        f"(Message + depth + reliability + durability) nor the "
                        f"service concept (Service)",
                    )
                )
    return findings


def _manifest_versions_by_domain(manifest) -> dict:
    path = Path(manifest)
    if not path.is_file():
        _warn(f"version_consistency: manifest not found at {path}; skipping the cross-check")
        return {}
    data = json.loads(path.read_text())
    result: dict = {}
    for entry in data.get("interfaces", []):
        domain = entry.get("domain")
        version = entry.get("version")
        if domain is not None and version is not None:
            result.setdefault(domain, set()).add(version)
    return result


def version_consistency(spec_dir: Path, manifest=None) -> list:
    """WARN on version-declaration problems in the domain headers.

    Flags a domain that does not declare exactly one `version{...}`, a domain
    whose MAJOR is not 0 (the standard is unstable at 0.x in this wave), and a
    manifest version that disagrees with the header version for that domain.
    """
    findings: list = []
    manifest_versions = _manifest_versions_by_domain(manifest) if manifest else {}
    for path in _domain_headers(spec_dir):
        h = parse_header(path)
        domain = path.stem
        anchor = h.version_lines[0] if h.version_lines else 1
        if len(h.versions) != 1:
            findings.append(
                Finding(
                    "WARN",
                    str(path),
                    anchor,
                    f"domain '{domain}' must declare exactly one "
                    f"'static constexpr Version version{{...}}' (found {len(h.versions)})",
                )
            )
            continue
        header_version = _version_str(h.versions[0])
        major = header_version.split(".")[0] if header_version else ""
        if major != "0":
            findings.append(
                Finding(
                    "WARN",
                    str(path),
                    anchor,
                    f"domain '{domain}' version '{header_version}' is not 0.x; the "
                    f"interface standard is unstable (0.x) in this wave",
                )
            )
        for manifest_version in sorted(manifest_versions.get(domain, set())):
            if manifest_version != header_version:
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        anchor,
                        f"domain '{domain}' manifest version '{manifest_version}' does "
                        f"not match header version '{header_version}'",
                    )
                )
    return findings


def _manifest_entries_by_key(manifest):
    """Index the committed manifest by (domain, interface), or None if unreadable."""
    path = Path(manifest)
    if not path.is_file():
        return None
    data = json.loads(path.read_text())
    entries = {}
    for entry in data.get("interfaces", []):
        domain, interface = entry.get("domain"), entry.get("interface")
        if domain is not None and interface is not None:
            entries[(domain, interface)] = entry
    return entries


def _parse_service_qos(spec_dir: Path):
    """Read the one QoS profile every service spec shares out of ``utils.hpp``.

    Derived rather than restated here: a copy of `depth`/`reliability`/`durability` in
    the linter would be one more place for the specs and the wire to drift apart, which
    is the drift this package exists to catch.
    """
    utils = Path(spec_dir) / _UTILS_HEADER
    if not utils.is_file():
        return None
    body = _group(_SERVICE_QOS_RE.search(_blank_comments(utils.read_text())))
    if body is None:
        return None
    qos = {
        "depth": _group(_DEPTH_VALUE_RE.search(body)),
        "reliability": _group(_RELIABILITY_VALUE_RE.search(body)),
        "durability": _group(_DURABILITY_VALUE_RE.search(body)),
    }
    return None if any(v is None for v in qos.values()) else qos


def _expected_qos(depth, reliability, durability):
    """Render a spec's QoS the way the generator's `to_string` does, or say why not."""
    if depth is None:
        return None, "declares no 'depth'"
    if reliability not in _RELIABILITY_NAMES:
        return None, f"declares a reliability policy the manifest cannot name: '{reliability}'"
    if durability not in _DURABILITY_NAMES:
        return None, f"declares a durability policy the manifest cannot name: '{durability}'"
    return {
        "history": _HISTORY,
        "depth": int(depth),
        "reliability": _RELIABILITY_NAMES[reliability],
        "durability": _DURABILITY_NAMES[durability],
    }, None


def qos_consistency(spec_dir: Path, manifest=None) -> list:
    """WARN when a registered spec's QoS disagrees with the manifest's `qos` block.

    `manifest_fresh` catches this too, but only where the generator binary is built; on
    the pre-commit path it skips. This check reads nothing but the headers and the
    committed JSON, so reliability and durability -- the two axes ROS 2 evaluates before
    it lets a publisher and a subscription talk at all -- stay verified there too.

    Topic specs carry their own QoS; service specs share the `service_qos` profile from
    `utils.hpp`. A spec absent from its namespace's `Specs` tuple is `spec_registered`'s
    finding, not this check's, so it is skipped rather than reported twice.
    """
    findings: list = []
    if manifest is None:
        return findings
    entries = _manifest_entries_by_key(manifest)
    if entries is None:
        _warn(f"qos_consistency: manifest not found at {manifest}; skipping the cross-check")
        return findings
    service_qos = _parse_service_qos(spec_dir)
    warned_no_service_qos = False

    for path in _domain_headers(spec_dir):
        header = parse_header(path)
        domain = path.stem
        registered = set(header.specs_members)
        for name, info in header.structs.items():
            if not info["has_name"] or name not in registered or not info["interface"]:
                continue
            if info["has_message"]:
                expected, error = _expected_qos(
                    info["depth"], info["reliability"], info["durability"]
                )
            elif info["has_service"]:
                if service_qos is None:
                    if not warned_no_service_qos:
                        _warn(
                            "qos_consistency: no 'service_qos' profile found in "
                            f"{_UTILS_HEADER}; skipping the service QoS cross-check"
                        )
                        warned_no_service_qos = True
                    continue
                expected, error = _expected_qos(**service_qos)
            else:
                continue  # neither topic nor service: interface_spec_concept's finding
            if error is not None:
                findings.append(Finding("WARN", str(path), info["line"], f"spec '{name}' {error}"))
                continue

            entry = entries.get((domain, info["interface"]))
            if entry is None:
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        info["line"],
                        f"spec '{name}' is registered in 'Specs' but the manifest has no "
                        f"'{domain}' entry for '{info['interface']}'; regenerate it",
                    )
                )
                continue
            actual = entry.get("qos")
            if actual is None:
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        info["line"],
                        f"spec '{name}' manifest entry records no 'qos' block; regenerate it",
                    )
                )
                continue
            # "specs", not "header": a service spec's QoS comes from the shared
            # `service_qos` profile in utils.hpp rather than from its own struct.
            keys = sorted(set(expected) | set(actual))
            drifted = [
                f"{k}: specs={expected.get(k)!r}, manifest={actual.get(k)!r}"
                for k in keys
                if expected.get(k) != actual.get(k)
            ]
            if drifted:
                findings.append(
                    Finding(
                        "WARN",
                        str(path),
                        info["line"],
                        f"spec '{name}' QoS disagrees with the manifest " f"({'; '.join(drifted)})",
                    )
                )
    return findings


def _warn(message: str) -> None:
    print(f"WARN {message}", file=sys.stderr)


def resolve_generator(generator=None):
    """Return a usable generator path from the argument or GENERATOR_ENV, else None."""
    candidate = generator or os.environ.get(GENERATOR_ENV)
    if not candidate:
        return None
    path = Path(candidate)
    return path if path.is_file() else None


def manifest_fresh(manifest=None, generator=None) -> list:
    """Rebuild the manifest with the M0.1 generator and diff the committed copy.

    In M0 this only RECORDS drift: it returns a WARN finding when the committed
    manifest is stale but never raises. It skips gracefully (returns []) when the
    generator or committed manifest is unavailable. M2 flips drift to a hard
    failure.
    """
    gen = resolve_generator(generator)
    if gen is None:
        _warn(
            "manifest_fresh: generator unavailable "
            f"(pass --generator or set {GENERATOR_ENV}); skipping in M0"
        )
        return []
    if manifest is None or not Path(manifest).is_file():
        _warn("manifest_fresh: committed manifest not found; skipping in M0")
        return []
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "manifest_under_test.json"
        proc = subprocess.run(
            [str(gen), str(out)],
            capture_output=True,
            text=True,
            check=False,
        )
        if proc.returncode != 0 or not out.is_file():
            _warn(f"manifest_fresh: generator failed (rc={proc.returncode}); skipping in M0")
            return []
        generated = out.read_text()
    committed = Path(manifest).read_text()
    if generated != committed:
        return [
            Finding(
                "WARN",
                str(manifest),
                1,
                "committed interface_manifest.json is stale: it differs from the "
                "generator output; regenerate it (M0 records only, M2 fails the build)",
            )
        ]
    return []
