# Glossary

- **NDT** — Normal Distributions Transform: registration by matching a scan to per-voxel Gaussians
  of the target map.
- **TP** — transform probability: the NDT score averaged over the cloud.
- **NVTL** — nearest-voxel transformation likelihood: the score against each point's nearest voxel,
  averaged.
- **Voxel / leaf** — a grid cell of the target and its fitted Gaussian (centroid + inverse
  covariance).
- **kd-tree** — spatial index over voxel centroids used for radius search.
- **Regularization** — optional longitudinal pull of the solution toward a reference (GNSS) pose.
- **`MatchScratch`** — the reused per-align workspace + last result; kept per task/thread.
- **`ArcSwap` / rcu** — the lock-free engine-state double-buffer; read-copy-update publishes a new
  state atomically.
- **WCET** — worst-case execution time; the bounded contract on the align path.
- **Host ports** — the `MapSource` / `OutputSink` / `Clock` traits the core is generic over.
- **`AwHost`** — the C-ABI vtable adapter of the Host ports (ROS side effects).
- **TPE** — Tree-Structured Parzen Estimator; the align-service pose-search sampler.
- **Oscillation** — consecutive direction inversions over the iteration trajectory; a local-minimum
  stop still counts as converged past a threshold.
- **No-ground** — scores/clouds computed with ground points filtered (a diagnostic variant).
- **Track A / Track B** — the std/ROS build vs the `no_std` kernel build.
- **`NDT_USE_RUST`** — the CMake option selecting the Rust backend over the legacy C++ engine.
