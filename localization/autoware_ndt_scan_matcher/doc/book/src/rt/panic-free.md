# Panic-free, bounded execution

The crate's lint hardening, seen through a real-time lens: the align path has no hidden abort, no
unbounded work, and no allocation.

## No panic source

Non-test code bans `unwrap` / `expect` / `panic` / `unreachable` / `todo` / `unimplemented` and
`indexing_slicing` / `string_slice` — so there is no implicit panic (which would abort or unwind) on
the RT path. Fallible operations return `Result`/`Option` and are handled; genuinely-dynamic
indexing uses `.get()`/iterators.

## No silent overflow

`[profile.release] overflow-checks = true` makes an integer overflow a **panic**, so integer
arithmetic must be explicit `checked_*` / `saturating_*` / `wrapping_*` — never bare `+`/`-`/`*`.
Float math (which does not panic) uses a scoped, documented allowance in the numeric kernels only.
Lossy `as` casts are banned in favor of `TryFrom`; a `Result` is never discarded with `let _`.

## Bounded and non-blocking

On the align path: loops are statically bounded (`max_iterations`, `MAX_NEIGHBORS`), buffers are
pre-sized and reused ([zero-alloc](zero-alloc.md)), and there is no blocking, no logging/formatting,
and no user callback. Together these give the [WCET contract](wcet.md) its teeth.

## Review

A `rust-realtime-review` accompanies each engine/align patch to keep these properties from
regressing; the lints themselves are enforced in CI (see
[Lint gates and suppression policy](../quality/hardening.md)).

> Source: `Cargo.toml` `[lints]` / `[profile.release]`; `src/ndt.rs`.
