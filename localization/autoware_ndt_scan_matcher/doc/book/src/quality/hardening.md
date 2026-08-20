# Lint gates and suppression policy

The crate holds itself to a zero-warning, zero-clippy-finding build, enforced by its `Cargo.toml`
`[lints]` table. Everything below is `-D warnings` in CI.

## The gates (`[lints]`)

- **`[lints.rust]`**: `warnings = "deny"`, `unsafe_code = "deny"`, `unsafe_op_in_unsafe_fn = "deny"`.
- **`[lints.clippy]` — denied**: `unwrap_used`, `expect_used`, `panic`, `unreachable`, `todo`,
  `unimplemented`, `indexing_slicing`, `string_slice`, `unwrap_in_result`, `arithmetic_side_effects`,
  `let_underscore_must_use`, `as_conversions`, `cast_possible_truncation`, `cast_possible_wrap`,
  `cast_sign_loss`, `float_cmp`, `lossy_float_literal`.
- **warn** (still errors under `-D warnings`): `cast_lossless`, `allow_attributes`,
  `pedantic` (priority `-1`); **`allow_attributes_without_reason = "deny"`** (every suppression needs
  a `reason`).
- **`[profile.release] overflow-checks = true`** — integer overflow panics, so integer arithmetic
  must be explicit `checked_*`/`saturating_*`/`wrapping_*` (see [Panic-free](../rt/panic-free.md)).

## Suppression policy

Suppression is the rare, self-cleaning exception: prefer `#[expect(…, reason = "…")]` over
`#[allow]`, at the **narrowest** scope (the function/expression that trips the lint), and only for a
lint on the project allowlist. **No module-wide `#![allow]` in production** — not even for numeric
kernels. Generated code (`ros_msgs`) and `#[cfg(test)]` modules may relax with one block `#[allow(…,
clippy::allow_attributes, reason = "…")]`.

Allowlist (production, per function, each with its condition):

| Lint(s)                            | Allowed only for                                                                                      |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `arithmetic_side_effects`          | f64/f32 **float** math (integer arithmetic must use checked/saturating)                               |
| `indexing_slicing`                 | fixed-size nalgebra `Vector`/`Matrix` indexing, or a provably-bounded index (e.g. `axis = depth % 3`) |
| `as_conversions`, `cast_*`         | a deliberate, documented conversion (the f32 `Matrix4f` parity pipeline)                              |
| readability `pedantic`/style lints | readability/parity in math kernels — never a safety lint                                              |

**Absolute-never in production:** `unwrap_used`, `expect_used`, `panic`, `unreachable`, `todo`,
`unimplemented`, `string_slice` — suppressing these would defeat the gates.

## Running

`cargo clippy --all-targets -- -D warnings` (std/default) and
`cargo clippy --no-default-features --features mt,awkernel_sync/std -- -D warnings` (multi-core
`no_std`); the single-core `no_std` compile is checked with the `cargo rustc … --target …-unknown-none`
gate. See [Build and test](../start/build-and-test.md).

> Source: `Cargo.toml` `[lints]` / `[profile.release]`.
