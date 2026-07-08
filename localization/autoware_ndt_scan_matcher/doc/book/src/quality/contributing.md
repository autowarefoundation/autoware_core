# Contributing and PR conventions

Conventions for changes to the port, especially for the upstream `autoware_core` PR.

Planned contents:

- **Sign off every commit** (`git commit -s`) — the upstream DCO check requires it.
- **No `Co-Authored-By` trailer** — an unsigned co-author fails DCO (broke PR #1217).
- Commit hygiene: one boundary/callback per commit; small, reviewable commits.
- The lint gate must be green (`cargo clippy -- -D warnings`) across the default and
  `no_std`/`mt` configs; rustfmt required.
- The upstream-divergence rule (notify + record + reproduce-not-fix; see
  [Divergences](../port/divergences.md)).
- Where docs live: this book (`doc/book/`) and the crate rustdoc.

> Status: outline (draft to be written).
> Source: the Autoware Core contribution guidelines (DCO), `Cargo.toml` `[lints]`.
