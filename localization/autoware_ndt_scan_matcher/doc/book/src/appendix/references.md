# References

## Project

- Autoware Core: <https://github.com/autowarefoundation/autoware>
- The package `README.md` — node I/O, parameters, and the regularization derivation.
- The crate rustdoc (`cargo doc --no-deps`) and the per-module docs in `src/`.
- Upstream fix: `autoware_core` PR #1217 — the NDT `h_ang` "d1" pitch² sign correction (see
  [Divergences from upstream](../port/divergences.md)).

## NDT algorithm

- P. Biber and W. Straßer, "The Normal Distributions Transform: a new approach to laser scan
  matching," IROS 2003.
- M. Magnusson, "The Three-Dimensional Normal-Distributions Transform — an Efficient Representation
  for Registration, Surface Analysis, and Loop Detection," PhD thesis, Örebro University, 2009.
