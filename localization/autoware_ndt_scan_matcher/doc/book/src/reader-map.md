# How to read this book

The book serves four audiences. Start where your goal sits; every chapter cross-links the
rest.

| You are…                               | Start with                                                                                                                                                   | Then                                                                                                     |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------- |
| **An upstream / PR reviewer**          | [Part IV — The C++ to Rust Port](port/verification.md), especially [Verification](port/verification.md) and [Divergences from upstream](port/divergences.md) | [Quality Gates](quality/hardening.md) (lints) → [System overview](arch/overview.md)                      |
| **A maintainer / contributor**         | [Part III — Architecture](arch/overview.md) (overview → FFI → engine → align)                                                                                | [Divergences](port/divergences.md), [Symbol map](port/symbol-map.md) → [Test taxonomy](quality/tests.md) |
| **A Rust crate user**                  | [Part II — Getting Started](start/build-and-test.md) (build, features, using the crate)                                                                      | [The NDT engine](arch/engine.md), [The align hot path](arch/align.md), and the generated rustdoc         |
| **An RT / `no_std` (kernel) engineer** | [Part V — Real-Time and no_std](rt/wcet.md) (WCET, zero-alloc, `mt`, panic-free)                                                                             | [Concurrency and interior mutability](arch/concurrency.md) → [lint gates](quality/hardening.md)          |

## Conventions

- **Code that is meant to compile** is shown as tested `rust` doctests copied from the crate's
  rustdoc. **Illustrative snippets and excerpts** use `rust,ignore` or `text` and are not compiled.
- **Paths** like `src/engine.rs` are relative to the crate root
  (`autoware_ndt_scan_matcher/autoware_ndt_scan_matcher_rs/`).
- **`Aw*`** names are C-ABI FFI types; **`Ndt*` / `ScanMatcher`** are Rust-side types.
- **TP** = transform probability, **NVTL** = nearest-voxel transformation likelihood
  (see the [Glossary](appendix/glossary.md)).
- Each chapter ends with a **Source** note listing the in-tree files it distills, so a reader can go
  from the prose to the authoritative code.
