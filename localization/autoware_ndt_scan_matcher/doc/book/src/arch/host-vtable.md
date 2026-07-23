# The Host abstraction and C vtables

The host abstraction — how Rust reaches ROS side effects — is a **portable Rust trait**. The C
vtable is just one adapter of it, so the same core runs under ROS, a kernel, or an async runtime.

## The Rust ports (`src/host.rs`)

```rust,ignore
pub trait MapSource { fn load(&self, center: [f64; 2], radius: f64) -> impl Future<Output = MapDelta>; }
pub trait OutputSink { fn publish_result(&self, result: &MatchResult); }
pub trait Clock      { fn now_ns(&self) -> i64; }
pub trait Host: MapSource + OutputSink + Clock {}   // blanket-impl'd
```

Portable node logic takes `host: &H` with `H: Host` and never names ROS. **Map loading is `async`**
(a ROS service future, a kernel DMA read); the engine **align hot path stays synchronous** and runs
between awaits — so a single-threaded executor can't deadlock on a blocking map call.

## Adapters

- **ROS** — a Rust `FfiHost` implements `Host` on top of the C vtable below.
- **Tokio** — `examples/tokio_ndt.rs`, the async reference over synthetic data.
- **Kernel** (future) — its own async runtime + flash/DMA map source.

## The C vtable (`src/ffi_host.rs`)

`AwHost` is a `#[repr(C)]` struct of C function pointers over an opaque `ctx` (the `NDTScanMatcher*`).
Rust never publishes directly; it requests side effects through the vtable:

```rust,ignore
#[repr(C)] pub struct AwHost {
    ctx: *mut c_void,
    now_ns: extern "C" fn(*mut c_void) -> i64,
    log: extern "C" fn(*mut c_void, level: i32, msg: *const u8, msg_len: usize),
    lookup_transform: extern "C" fn(*mut c_void, target: AwStr, source: AwStr,
                                    out_matrix4x4_row_major: *mut f32) -> bool,
    publish_pose:       extern "C" fn(/* topic, stamp, *AwPose, *cov6x6 (null => PoseStamped) */),
    publish_pose_array: extern "C" fn(/* topic, stamp, *AwPose, n */),
    publish_marker:     extern "C" fn(/* iteration-trajectory MarkerArray */),
    // … pointcloud / float / int / tf publishers …
}
```

Each callback is POD-in; the C++ trampoline builds the ROS message, knows the static frame ids, and
is `catch(...)`-guarded so a publish exception never unwinds across the boundary. Keep the vtable to
**ROS side effects only** — clock, TF, publish, log, map-load — never algorithm state (that is
Rust-owned; see [Scope](../concepts/scope.md)).

> Source: `src/host.rs`, `src/ffi_host.rs`, `src/node_map_update.rs`, `examples/tokio_ndt.rs`.
