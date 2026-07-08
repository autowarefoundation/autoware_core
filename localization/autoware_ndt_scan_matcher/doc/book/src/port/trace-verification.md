# Trace-based state-machine verification

Proving observable equivalence for control-flow-heavy paths via abstract traces.

Planned contents:

- The trace-based state-machine verification workflow: C++ baseline → spec-level state machine →
  abstract trace → instrument both sides → differential trace diff.
- The align-service semantic trace ABI (decision events, search summary, response summary) — store
  only semantic fields, not ROS byte dumps.
- Splitting verification by determinism: exact trace checks for deterministic control flow,
  tolerance checks for align outcomes, property/statistical checks for TPE search quality.
- Establishing tolerances from measured C++ baseline self-variance.

> Status: outline (draft to be written).
> Source: `src/node_align_service.rs` (trace events + the align-service decision functions).
