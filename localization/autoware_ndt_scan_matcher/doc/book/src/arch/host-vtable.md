# The Host abstraction and C vtables

The host abstraction is a **portable Rust trait**; the C vtable is one adapter of it.

Planned contents:

- The trait set in `src/host.rs`: `MapSource` (async `load`), `OutputSink`, `Clock`, blanket
  `Host`.
- Adapters: the ROS `FfiHost` over the `AwHost` C vtable; the Tokio reference
  (`examples/tokio_ndt.rs`); the future kernel adapter.
- The `AwHost` `#[repr(C)]` vtable of C function pointers over an opaque `ctx` (= `NDTScanMatcher*`):
  `now_ns`, `lookup_transform`, `publish_*`, `request_map_delta`, `log`.
- **Async I/O, synchronous align** — why `MapSource::load` is async and the align stays sync
  (single-threaded-executor deadlock avoidance); `apply_map_update` staging flow.
- Good vs bad Host responsibilities; the transitional node-state setters that were removed.

> Status: outline (draft to be written).
> Source: `src/host.rs`, `src/ffi_host.rs`, `src/node_map_update.rs`, `examples/tokio_ndt.rs`.
