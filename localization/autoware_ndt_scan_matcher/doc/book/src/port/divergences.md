# Divergences from upstream

Porting diffs the C++ and Rust implementations directly, which makes it the best opportunity to
find **upstream bugs**. The port's policy is deliberate and may be surprising: when the C++
reference diverges from expected-correct behavior, the Rust port **reproduces the C++ behavior**
rather than fixing it locally.

## Why reproduce, not fix

The differential test against C++ is the oracle (see [Verification](verification.md)). If the Rust
port silently "corrected" a C++ bug, the differential test would go red and we would lose the
ability to prove equivalence. So the correct fix belongs **upstream** (in pcl / Autoware), and the
port keeps the C++ behavior verbatim until that upstream fix lands.

## The process for a discovered divergence

1. **Notify the user immediately** — what / where / evidence / impact. Never silently absorb it.
2. **Record it** in this chapter's divergence list — one entry per finding: location · type ·
   evidence · correct value · impact · decision · revisit trigger · upstream link · verification.
3. **Reproduce, don't fix locally.** Mark the site in code (`PORT-QUIRK`) and pin it with a test,
   so an accidental "fix" fails loudly.
4. **Fix upstream.** File/draft an upstream issue and link it.

## Follow the fix when upstream lands

Reproduce-not-fix is temporary per finding. Once the upstream fix lands, the port **follows it**:
update the Rust to match the corrected C++, widen or re-point the pinning test to the corrected
value, and move the entry to *Resolved* below. Following the fix restores `ON == OFF` (both now
compute the corrected value).

## Resolved

- **NDT `h_ang` "d1" sign bug — fixed upstream (PR #1217), port followed.**
  `computeAngleDerivatives()` built the angular Hessian table `h_ang` with the pitch² x-coefficient
  as `+sy`, where the exact second derivative is `-sy` (row 6, "d1"). It affected only the Newton
  search direction, not the optimum. The port originally reproduced `+sy` to stay bit-identical with
  the then-buggy C++; when the upstream fix landed the port followed it (`src/derivatives.rs`, now
  `-sy`). With the sign corrected the pcl NDT Hessian is the **exact** analytic Hessian, so the full
  6×6 Hessian now finite-difference-validates — the FD tests
  (`update_derivatives_gradient_and_hessian_match_finite_difference` in `src/derivatives.rs`,
  `compute_derivatives_matches_finite_difference` in `src/ndt.rs`) cover all six rows and pin the
  corrected sign directly (they fail on `+sy`).

## Intentional differences

The align-service TPE uses a deterministic Rust-owned sampler instead of libstdc++'s
implementation-defined `std::normal_distribution` sequence, because that sequence is not portable
(this is why exact candidate-trace equivalence for the TPE search is out of scope; see
[Verification](verification.md)).

> Source: `src/derivatives.rs` (the `h_ang` table + FD tests); upstream issue/PR links.
