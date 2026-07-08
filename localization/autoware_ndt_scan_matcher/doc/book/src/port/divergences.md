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

## Known items

- **pcl NDT Hessian is approximate.** The pcl NDT implementation computes an approximate Hessian;
  the port matches the C++ result, not exact analytic math. This matters at fine tolerances (it
  surfaces around the covariance/E4d-level checks). The port mirrors it and pins the behavior.
- **NDT `h_ang` "d1" sign bug.** Found during the port and **fixed upstream** (PR #1217).

Intentional (non-bug) differences exist too — e.g. the align-service TPE uses a deterministic
Rust-owned sampler instead of libstdc++'s implementation-defined `std::normal_distribution`
sequence, because that sequence is not portable (this is why exact candidate-trace equivalence for
the TPE search is out of scope; see [Verification](verification.md)).

> Source: in-code `PORT-QUIRK` markers pinned by tests; upstream issue links.
