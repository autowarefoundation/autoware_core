#!/usr/bin/env python3
# Copyright 2024 Autoware Foundation
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

"""Render a self-contained HTML report from ndt_bench_replay's JSON (see plan/ndt_bench.md).

Stdlib only. Computes latency distribution stats (p50/p95/p99/max/mean/stddev) per engine, a
C++-vs-Rust speedup, and inline-SVG histograms. Output is one .html file with inline CSS (no CDN,
no external assets), light/dark aware. Opens directly in a browser.

Usage: gen_report.py <bench.json> [report.html]
"""

import html
import json
import math
import sys


def percentile(sorted_xs, q):
    """Linear-interpolated percentile (q in [0,1]) of an already-sorted list."""
    if not sorted_xs:
        return 0.0
    if len(sorted_xs) == 1:
        return sorted_xs[0]
    pos = q * (len(sorted_xs) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    frac = pos - lo
    return sorted_xs[lo] * (1.0 - frac) + sorted_xs[hi] * frac


def stats(samples):
    xs = sorted(samples)
    n = len(xs)
    mean = sum(xs) / n if n else 0.0
    var = sum((x - mean) ** 2 for x in xs) / n if n else 0.0
    return {
        "n": n,
        "min": xs[0] if n else 0.0,
        "p50": percentile(xs, 0.50),
        "p95": percentile(xs, 0.95),
        "p99": percentile(xs, 0.99),
        "max": xs[-1] if n else 0.0,
        "mean": mean,
        "stddev": math.sqrt(var),
    }


def histogram(samples, bins, lo, hi):
    counts = [0] * bins
    if hi <= lo:
        return counts
    width = (hi - lo) / bins
    for x in samples:
        k = int((x - lo) / width)
        if k < 0:
            k = 0
        if k >= bins:
            k = bins - 1
        counts[k] += 1
    return counts


def svg_histogram(samples, lo, hi, color, bins=40, w=520, h=140):
    """Inline-SVG histogram bars over a shared [lo, hi] x-range."""
    counts = histogram(samples, bins, lo, hi)
    peak = max(counts) if counts and max(counts) > 0 else 1
    bw = w / bins
    bars = []
    for i, c in enumerate(counts):
        bh = (c / peak) * (h - 18)
        x = i * bw
        y = (h - 18) - bh
        bars.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{bw - 1:.2f}" height="{bh:.2f}" '
            f'fill="{color}" rx="1"/>'
        )
    # x-axis ticks (lo, mid, hi in ms)
    mid = (lo + hi) / 2.0
    ticks = "".join(
        f'<text x="{tx:.1f}" y="{h - 4}" font-size="10" fill="currentColor" '
        f'opacity="0.6" text-anchor="{anchor}">{val:.3f}</text>'
        for tx, anchor, val in (
            (2, "start", lo),
            (w / 2, "middle", mid),
            (w - 2, "end", hi),
        )
    )
    return (
        f'<svg viewBox="0 0 {w} {h}" width="100%" preserveAspectRatio="none" '
        f'role="img">{"".join(bars)}{ticks}</svg>'
    )


CSS = """
:root { color-scheme: light dark; --bg:#fff; --fg:#1a1a1a; --muted:#666; --line:#e3e3e3;
        --cpp:#d9822b; --rust:#4b6bfb; --card:#fafafa; --good:#1a7f37; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#0f1115; --fg:#e6e6e6; --muted:#9aa0a6; --line:#2a2d34; --cpp:#e0993e;
          --rust:#7a92ff; --card:#161922; --good:#4ac26b; } }
* { box-sizing: border-box; }
body { margin:0; padding:32px; background:var(--bg); color:var(--fg);
       font:15px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
h1 { font-size:22px; margin:0 0 4px; } h2 { font-size:16px; margin:28px 0 10px; }
.sub { color:var(--muted); margin:0 0 24px; font-size:13px; }
.meta { display:flex; flex-wrap:wrap; gap:8px 20px; margin:0 0 24px; font-size:13px;
        color:var(--muted); }
.meta b { color:var(--fg); font-weight:600; }
table { border-collapse:collapse; width:100%; max-width:820px; font-variant-numeric:tabular-nums; }
th,td { padding:8px 12px; text-align:right; border-bottom:1px solid var(--line); }
th:first-child,td:first-child { text-align:left; }
thead th { font-size:12px; text-transform:uppercase; letter-spacing:.04em; color:var(--muted); }
.dot { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:7px;
       vertical-align:middle; }
.cpp { color:var(--cpp); } .rust { color:var(--rust); }
.bg-cpp { background:var(--cpp); } .bg-rust { background:var(--rust); }
.speedup { font-size:15px; margin:6px 0 0; }
.speedup b { font-size:19px; color:var(--good); }
.charts { display:grid; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); gap:20px;
          max-width:1100px; }
.card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:14px 16px; }
.card h3 { margin:0 0 2px; font-size:14px; } .card .cap { color:var(--muted); font-size:12px;
          margin:0 0 10px; }
.foot { color:var(--muted); font-size:12px; margin-top:28px; }
"""


def fmt(v):
    return f"{v:.4f}"


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: gen_report.py <bench.json> [report.html]")
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else "report.html"
    with open(src, encoding="utf-8") as fh:
        data = json.load(fh)

    meta = data.get("meta", {})
    engines = data["engines"]
    order = [k for k in ("cpp", "rust") if k in engines] + [
        k for k in engines if k not in ("cpp", "rust")
    ]
    st = {k: stats(engines[k]["samples_ms"]) for k in order}

    # shared x-range for comparable histograms
    all_s = [x for k in order for x in engines[k]["samples_ms"]]
    lo, hi = (min(all_s), max(all_s)) if all_s else (0.0, 1.0)
    if hi <= lo:
        hi = lo + 1.0
    colorvar = {"cpp": "var(--cpp)", "rust": "var(--rust)"}

    rows = []
    for k in order:
        s = st[k]
        rows.append(
            "<tr><td>{lab}</td><td>{n}</td><td>{p50}</td><td>{p95}</td><td>{p99}</td>"
            "<td>{mx}</td><td>{mean}</td><td>{sd}</td><td>{it}</td></tr>".format(
                lab='<span class="dot bg-{c}"></span>{name}'.format(
                    c=k, name=html.escape(engines[k].get("label", k))
                ),
                n=s["n"],
                p50=fmt(s["p50"]),
                p95=fmt(s["p95"]),
                p99=fmt(s["p99"]),
                mx=fmt(s["max"]),
                mean=fmt(s["mean"]),
                sd=fmt(s["stddev"]),
                it=engines[k].get("iteration_num", "-"),
            )
        )

    speedup_html = ""
    if "cpp" in st and "rust" in st and st["rust"]["p50"] > 0:
        ratio = st["cpp"]["p50"] / st["rust"]["p50"]
        faster = "Rust" if ratio > 1 else "C++"
        factor = ratio if ratio > 1 else (1.0 / ratio if ratio > 0 else 0.0)
        speedup_html = (
            f'<p class="speedup">At the median, <span class="{faster.lower() if faster=="Rust" else "cpp"}">'
            f"{faster}</span> is <b>{factor:.2f}×</b> "
            f"{'faster' if factor>=1 else ''} (C++ p50 {fmt(st['cpp']['p50'])} ms vs "
            f"Rust p50 {fmt(st['rust']['p50'])} ms).</p>"
        )

    charts = []
    for k in order:
        s = st[k]
        charts.append(
            '<div class="card"><h3 class="{c}">{name}</h3>'
            '<p class="cap">per-align latency (ms) · p50 {p50} · p99 {p99} · max {mx}</p>'
            "{svg}</div>".format(
                c=k,
                name=html.escape(engines[k].get("label", k)),
                p50=fmt(s["p50"]),
                p99=fmt(s["p99"]),
                mx=fmt(s["max"]),
                svg=svg_histogram(engines[k]["samples_ms"], lo, hi, colorvar.get(k, "gray")),
            )
        )

    meta_items = [
        ("Benchmark", data.get("benchmark", "NDT align replay")),
        ("Points", meta.get("n_points")),
        ("Aligns / engine", meta.get("iters")),
        ("Warmup", meta.get("warmup")),
        ("Resolution", meta.get("resolution")),
        ("max_iterations", meta.get("max_iterations")),
        ("num_threads", meta.get("num_threads")),
        ("Clock", meta.get("clock")),
    ]
    meta_html = "".join(
        f"<span><b>{html.escape(str(v))}</b> {html.escape(k)}</span>"
        for k, v in meta_items
        if v is not None
    )

    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NDT bench — C++ vs Rust</title><style>{CSS}</style></head><body>
<h1>NDT Scan Matcher — C++ vs Rust</h1>
<p class="sub">{html.escape(meta.get("note", "align loop only; map+kdtree built once per engine"))}</p>
<div class="meta">{meta_html}</div>
{speedup_html}
<h2>Latency distribution (ms)</h2>
<table><thead><tr><th>Engine</th><th>N</th><th>p50</th><th>p95</th><th>p99</th><th>max</th>
<th>mean</th><th>stddev</th><th>iters</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table>
<h2>Per-align latency histograms</h2>
<div class="charts">{''.join(charts)}</div>
<p class="foot">Lower is better. Histograms share the x-range [{fmt(lo)}, {fmt(hi)}] ms.
Generated by bench/gen_report.py from {html.escape(src)}.</p>
</body></html>
"""
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(doc)
    print(f"wrote {dst}")


if __name__ == "__main__":
    main()
