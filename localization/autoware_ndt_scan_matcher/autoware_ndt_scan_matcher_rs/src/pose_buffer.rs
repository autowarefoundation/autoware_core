// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! `no_std` port of `autoware_localization_util::SmartPoseBuffer` — a time-ordered buffer of stamped
//! poses-with-covariance that linearly interpolates a pose at a query time.
//!
//! Ported operation-for-operation from `smart_pose_buffer.cpp` + `util_func.cpp` so it differential-
//! tests bit-close against the C++ original: `push_back` clears the buffer on a backward stamp;
//! `interpolate` brackets the query time, validates both ends against the time-tolerance and the pair
//! against the distance-tolerance, then extrapolates from the older pose using the pair's **twist**
//! (linear in position, **linear in roll/pitch/yaw** with radian-normalized diffs — not slerp) and
//! **copies the older pose's covariance** unchanged; `pop_old` drops entries strictly older than the
//! query time. RPY↔quaternion use the ZYX Tait-Bryan convention (matching `tf2` `getRPY`/`setRPY`,
//! realized here via nalgebra `UnitQuaternion`).

use alloc::collections::VecDeque;

use nalgebra::{Quaternion, UnitQuaternion};

const TWO_PI: f64 = core::f64::consts::TAU;
const PI: f64 = core::f64::consts::PI;

/// A stamped pose with covariance held in the buffer. `orientation` is `[x, y, z, w]`; `covariance`
/// is the row-major 6x6.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedPoseWithCov {
    pub stamp_ns: i64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
}

/// The interpolation output: the pose at the query time (covariance carried from the older bracket
/// entry — the C++ does not interpolate covariance), plus the two bracket entries themselves (`old`
/// = last entry ≤ target, `new_entry` = first entry > target), which the C++ `InterpolateResult`
/// exposes as `old_pose`/`new_pose` and the sensor callback publishes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InterpolateResult {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
    pub old: TimedPoseWithCov,
    pub new_entry: TimedPoseWithCov,
}

/// A time-ordered pose buffer with the two `SmartPoseBuffer` validation tolerances.
#[derive(Clone, Debug)]
pub struct PoseBuffer {
    entries: VecDeque<TimedPoseWithCov>,
    timeout_sec: f64,
    distance_tol_m: f64,
}

impl PoseBuffer {
    #[must_use]
    pub fn new(timeout_sec: f64, distance_tol_m: f64) -> Self {
        Self {
            entries: VecDeque::new(),
            timeout_sec,
            distance_tol_m,
        }
    }

    /// Number of buffered entries.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Append `e`, enforcing chronological order: if its stamp precedes the current back (a backward
    /// time jump, e.g. a bag replay), the whole buffer is cleared first (mirrors the C++).
    pub fn push_back(&mut self, e: TimedPoseWithCov) {
        if let Some(back) = self.entries.back()
            && e.stamp_ns < back.stamp_ns
        {
            self.entries.clear();
        }
        self.entries.push_back(e);
    }

    /// Drop every entry with `stamp_ns < target_ns` (front-to-back), mirroring `pop_old`.
    pub fn pop_old(&mut self, target_ns: i64) {
        while let Some(front) = self.entries.front() {
            if front.stamp_ns >= target_ns {
                break;
            }
            self.entries.pop_front();
        }
    }

    /// Interpolate the pose at `target_ns`. Returns `None` (mirroring the C++ `std::nullopt`) when:
    /// fewer than 2 entries; `target_ns` precedes the oldest entry; or either bracket end fails the
    /// time-tolerance, or the bracket pair fails the distance-tolerance.
    #[must_use]
    pub fn interpolate(&self, target_ns: i64) -> Option<InterpolateResult> {
        if self.entries.len() < 2 {
            return None;
        }
        let time_first = self.entries.front()?.stamp_ns;
        if target_ns < time_first {
            return None;
        }
        // Bracket: `old` = last entry with stamp <= target; `new` = first entry with stamp > target
        // (or the last entry when target is at/after the back). Verbatim from the C++ loop.
        let mut old = *self.entries.front()?;
        let mut new = old;
        for e in &self.entries {
            new = *e;
            if e.stamp_ns > target_ns {
                break;
            }
            old = *e;
        }
        if !within_timeout(old.stamp_ns, target_ns, self.timeout_sec)
            || !within_timeout(new.stamp_ns, target_ns, self.timeout_sec)
            || !within_distance(&old.position, &new.position, self.distance_tol_m)
        {
            return None;
        }
        Some(interpolate_pose(&old, &new, target_ns))
    }
}

/// ns → seconds. The `i64` nanosecond magnitudes of realtime stamps are far inside f64's exact-integer
/// range (2^53 ns ≈ 104 days), so the precision loss is immaterial — this mirrors
/// `rclcpp::Duration::seconds()`.
#[expect(
    clippy::as_conversions,
    clippy::cast_precision_loss,
    reason = "ns->seconds; realtime stamp magnitudes are well within f64's exact-integer range"
)]
fn ns_to_sec(ns: i64) -> f64 {
    (ns as f64) * 1e-9
}

/// `|stamp - target|` in seconds `< timeout` (the C++ `validate_time_stamp_difference`, strict `<`).
fn within_timeout(stamp_ns: i64, target_ns: i64, timeout_sec: f64) -> bool {
    libm::fabs(ns_to_sec(stamp_ns.saturating_sub(target_ns))) < timeout_sec
}

/// Euclidean distance between the two positions `< tol` (the C++ `validate_position_difference`).
fn within_distance(a: &[f64; 3], b: &[f64; 3], tol_m: f64) -> bool {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    libm::sqrt(dx * dx + dy * dy + dz * dz) < tol_m
}

/// Quaternion `[x, y, z, w]` → `(roll, pitch, yaw)` (ZYX Tait-Bryan; matches `tf2` `getRPY`).
fn quat_to_rpy(q: [f64; 4]) -> (f64, f64, f64) {
    UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2])).euler_angles()
}

/// `(roll, pitch, yaw)` → quaternion `[x, y, z, w]` (matches `tf2::Quaternion::setRPY`).
fn rpy_to_quat(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let q = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let q = q.quaternion();
    [q.i, q.j, q.k, q.w]
}

/// Normalize a radian difference into `[-pi, pi)` (the C++ `autoware_utils_math::normalize_radian`).
fn normalize_radian(rad: f64) -> f64 {
    let value = libm::fmod(rad, TWO_PI);
    if (-PI..PI).contains(&value) {
        value
    } else {
        value - libm::copysign(TWO_PI, value)
    }
}

/// Port of `util_func.cpp` `interpolate_pose`: extrapolate from `old` by the pair's twist over
/// `dt = target - old`. Position linear; orientation linear in RPY (radian-normalized diffs);
/// covariance carried from `old`. Zero twist when the bracket stamps coincide; a zero stamp anywhere
/// yields a zero pose (mirrors the C++ empty-`PoseStamped` guard).
fn interpolate_pose(old: &TimedPoseWithCov, new: &TimedPoseWithCov, target_ns: i64) -> InterpolateResult {
    if old.stamp_ns == 0 || new.stamp_ns == 0 || target_ns == 0 {
        return InterpolateResult {
            position: [0.0; 3],
            orientation: [0.0; 4],
            covariance: old.covariance,
            old: *old,
            new_entry: *new,
        };
    }
    let (roll_a, pitch_a, yaw_a) = quat_to_rpy(old.orientation);
    let dt = ns_to_sec(target_ns.saturating_sub(old.stamp_ns));

    // calc_twist: per-second linear + angular rates, or zero when the bracket stamps coincide
    // (the C++ `dt_s == 0` guard, tested on the integer stamps to avoid a float compare).
    let (vx, vy, vz, wr, wp, wy) = if new.stamp_ns == old.stamp_ns {
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    } else {
        let dt_ab = ns_to_sec(new.stamp_ns.saturating_sub(old.stamp_ns));
        let (roll_b, pitch_b, yaw_b) = quat_to_rpy(new.orientation);
        (
            (new.position[0] - old.position[0]) / dt_ab,
            (new.position[1] - old.position[1]) / dt_ab,
            (new.position[2] - old.position[2]) / dt_ab,
            normalize_radian(roll_b - roll_a) / dt_ab,
            normalize_radian(pitch_b - pitch_a) / dt_ab,
            normalize_radian(yaw_b - yaw_a) / dt_ab,
        )
    };

    let position = [
        old.position[0] + vx * dt,
        old.position[1] + vy * dt,
        old.position[2] + vz * dt,
    ];
    let orientation = rpy_to_quat(roll_a + wr * dt, pitch_a + wp * dt, yaw_a + wy * dt);
    InterpolateResult {
        position,
        orientation,
        covariance: old.covariance,
        old: *old,
        new_entry: *new,
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::expect_used,
    clippy::allow_attributes,
    reason = "test code: exact-equal asserts on deterministic math + fixed-size index reads"
)]
mod tests {
    use super::*;

    fn entry(stamp_ns: i64, x: f64, y: f64, yaw: f64) -> TimedPoseWithCov {
        let mut covariance = [0.0_f64; 36];
        covariance[0] = x; // a recognizable marker so the "covariance from old" rule is testable
        TimedPoseWithCov {
            stamp_ns,
            position: [x, y, 0.0],
            orientation: rpy_to_quat(0.0, 0.0, yaw),
            covariance,
        }
    }

    fn buf() -> PoseBuffer {
        PoseBuffer::new(1000.0, 1000.0)
    }

    #[test]
    fn fewer_than_two_entries_is_none() {
        let mut b = buf();
        assert!(b.interpolate(5).is_none());
        b.push_back(entry(10, 0.0, 0.0, 0.0));
        assert!(b.interpolate(10).is_none());
    }

    #[test]
    fn target_before_first_is_none() {
        let mut b = buf();
        b.push_back(entry(10, 0.0, 0.0, 0.0));
        b.push_back(entry(20, 1.0, 0.0, 0.0));
        assert!(b.interpolate(5).is_none());
    }

    #[test]
    fn midpoint_interpolates_position_and_yaw() {
        let mut b = buf();
        b.push_back(entry(1, 0.0, 0.0, 0.0)); // nonzero stamp (avoid the stamp==0 guard)
        b.push_back(entry(2_000_000_001, 2.0, 4.0, 1.0));
        let r = b.interpolate(1_000_000_001).expect("valid");
        assert!((r.position[0] - 1.0).abs() < 1e-9);
        assert!((r.position[1] - 2.0).abs() < 1e-9);
        let (_, _, yaw) = quat_to_rpy(r.orientation);
        assert!((yaw - 0.5).abs() < 1e-9);
        // The bracket entries are exposed (old <= target < new) for the sensor callback's publish.
        assert_eq!(r.old.stamp_ns, 1);
        assert_eq!(r.new_entry.stamp_ns, 2_000_000_001);
        assert_eq!(r.old.position[0], 0.0);
        assert_eq!(r.new_entry.position[0], 2.0);
    }

    #[test]
    fn covariance_comes_from_old_entry() {
        let mut b = buf();
        b.push_back(entry(1, 7.0, 0.0, 0.0)); // covariance[0] = 7.0
        b.push_back(entry(2_000_000_001, 9.0, 0.0, 0.0)); // covariance[0] = 9.0
        let r = b.interpolate(1_000_000_001).expect("valid");
        assert_eq!(r.covariance[0], 7.0); // old, not new, not interpolated
    }

    #[test]
    fn push_back_clears_on_backward_stamp() {
        let mut b = buf();
        b.push_back(entry(100, 0.0, 0.0, 0.0));
        b.push_back(entry(200, 1.0, 0.0, 0.0));
        b.push_back(entry(50, 9.0, 0.0, 0.0)); // backward → clears, then pushes
        assert_eq!(b.len(), 1);
    }

    #[test]
    fn pop_old_drops_strictly_older() {
        let mut b = buf();
        b.push_back(entry(10, 0.0, 0.0, 0.0));
        b.push_back(entry(20, 1.0, 0.0, 0.0));
        b.push_back(entry(30, 2.0, 0.0, 0.0));
        b.pop_old(20);
        assert_eq!(b.len(), 2); // 10 dropped, 20 kept (>= target)
    }

    #[test]
    fn time_tolerance_rejects_stale_bracket() {
        let mut b = PoseBuffer::new(0.5, 1000.0); // 0.5 s timeout
        b.push_back(entry(1, 0.0, 0.0, 0.0));
        b.push_back(entry(2_000_000_001, 1.0, 0.0, 0.0)); // ~2 s apart
        // target near the new end → old bracket is ~2 s stale → reject
        assert!(b.interpolate(2_000_000_000).is_none());
    }

    #[test]
    fn distance_tolerance_rejects_large_jump() {
        let mut b = PoseBuffer::new(1000.0, 1.0); // 1 m tolerance
        b.push_back(entry(1, 0.0, 0.0, 0.0));
        b.push_back(entry(2_000_000_001, 5.0, 0.0, 0.0)); // 5 m apart
        assert!(b.interpolate(1_000_000_001).is_none());
    }
}
