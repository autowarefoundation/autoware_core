# Trace-based state-machine verification

For control-flow-heavy paths — notably the align service — a byte-for-byte output diff is too brittle
and too coarse. Instead the port proves equivalence with an **abstract semantic trace**: a spec-level
state machine emitting only the fields that matter, instrumented on both sides and diffed.

## The workflow

1. Establish the C++ baseline behavior.
2. Model it as a spec-level state machine (the decision points, not the ROS plumbing).
3. Emit an abstract trace of semantic events from both C++ and Rust.
4. Differentially test the traces; triage divergences; add regression tests.

## The align-service trace ABI

`node_align_service` carries a `#[repr(C)]` trace buffer (`AwNdtAlignServiceTrace` + typed
`AwNdtAlignServiceTraceEvent`s) written by `append_trace_event` / `append_search_summary_trace` /
`append_response_trace`. Events record **semantic** fields only — the gate decision
(status/success/reliable/valid), the diagnostic level + message kind, the search summary (requested
/ evaluated particles, best iteration/score, publish counts), and the response payload — never raw
ROS message bytes. A null trace pointer is a no-op, so production carries no cost; tests install a
buffer and assert the exact event sequence.

## Split by determinism

Deterministic control flow (request validation, availability gates, initial-pose branch selection,
response packaging, and the host side-effect summary: topic/type/count) gets **exact** trace checks.
The TPE/NDT search — where libstdc++'s sampler makes exact candidate traces unstable — gets
tolerance checks on the align outcome and property/statistical checks on search quality instead (see
[Verification](verification.md)). Tolerances are established from measured C++ baseline self-variance,
not invented up front.

> Source: `src/node_align_service.rs` (the trace events + the deterministic decision functions).
