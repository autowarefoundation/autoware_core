# Lint gates and suppression policy

The zero-warning / zero-clippy build the crate holds itself to, and the narrow allowlist for
suppressions.

Planned contents:

- The banned constructs in non-test code: `unwrap`/`expect`/`panic`/`unreachable`/`todo`/
  `unimplemented`, indexing/slicing, silently-overflowing arithmetic, lossy `as` casts,
  `let _`-discarded `Result`.
- The `[lints]` table in `Cargo.toml` and `overflow-checks = true`.
- The suppression policy: `#[expect(…, reason = …)]` over `#[allow]`, narrowest scope, **no
  module-wide `#![allow]` in production**; `allow_attributes` / `allow_attributes_without_reason`.
- The project allowlist (float `arithmetic_side_effects`, fixed-size nalgebra `indexing_slicing`,
  documented casts, readability pedantic lints) — never a safety lint.
- Generated (`ros_msgs`) and `#[cfg(test)]` modules may relax with one block `#[allow]`.

> Status: outline (draft to be written).
> Source: `Cargo.toml` `[lints]` (the enforced lint set + suppression allowlist).
