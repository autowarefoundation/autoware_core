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

//! Pure ports of `autoware::ndt_scan_matcher` C++ helper functions.
//!
//! These mirror `src/ndt_scan_matcher_helper.cpp` exactly so the existing C++ gtest
//! (`test/test_ndt_scan_matcher_helper.cpp`) acts as the differential oracle.

/// Rotate the 3x3 position block of a 6x6 (row-major, 36-element) pose covariance by the
/// given rotation matrix, i.e. compute `R * C * R^T` for the upper-left 3x3 block while
/// leaving the remaining entries untouched.
///
/// `rot` is a row-major 3x3 matrix (`rot[3*r + c] = R(r, c)`).
#[must_use]
pub fn rotate_covariance(src: &[f64; 36], rot: &[f64; 9]) -> [f64; 36] {
    let &[r00, r01, r02, r10, r11, r12, r20, r21, r22] = rot;

    // 3x3 position block lives at row-major indices {0,1,2 / 6,7,8 / 12,13,14}; the `a*`
    // bindings are the untouched entries we pass straight through.
    let &[
        s00,
        s01,
        s02,
        a3,
        a4,
        a5,
        s10,
        s11,
        s12,
        a9,
        a10,
        a11,
        s20,
        s21,
        s22,
        a15,
        a16,
        a17,
        a18,
        a19,
        a20,
        a21,
        a22,
        a23,
        a24,
        a25,
        a26,
        a27,
        a28,
        a29,
        a30,
        a31,
        a32,
        a33,
        a34,
        a35,
    ] = src;

    // M = R * C
    let m00 = r00 * s00 + r01 * s10 + r02 * s20;
    let m01 = r00 * s01 + r01 * s11 + r02 * s21;
    let m02 = r00 * s02 + r01 * s12 + r02 * s22;
    let m10 = r10 * s00 + r11 * s10 + r12 * s20;
    let m11 = r10 * s01 + r11 * s11 + r12 * s21;
    let m12 = r10 * s02 + r11 * s12 + r12 * s22;
    let m20 = r20 * s00 + r21 * s10 + r22 * s20;
    let m21 = r20 * s01 + r21 * s11 + r22 * s21;
    let m22 = r20 * s02 + r21 * s12 + r22 * s22;

    // out_block = M * R^T  (out(i,j) = sum_k M(i,k) * R(j,k))
    let o00 = m00 * r00 + m01 * r01 + m02 * r02;
    let o01 = m00 * r10 + m01 * r11 + m02 * r12;
    let o02 = m00 * r20 + m01 * r21 + m02 * r22;
    let o10 = m10 * r00 + m11 * r01 + m12 * r02;
    let o11 = m10 * r10 + m11 * r11 + m12 * r12;
    let o12 = m10 * r20 + m11 * r21 + m12 * r22;
    let o20 = m20 * r00 + m21 * r01 + m22 * r02;
    let o21 = m20 * r10 + m21 * r11 + m22 * r12;
    let o22 = m20 * r20 + m21 * r21 + m22 * r22;

    [
        o00, o01, o02, a3, a4, a5, o10, o11, o12, a9, a10, a11, o20, o21, o22, a15, a16, a17, a18,
        a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35,
    ]
}

const INVERSION_VECTOR_THRESHOLD: f64 = -0.9;

fn norm3(v: &[f64; 3]) -> f64 {
    let &[x, y, z] = v;
    // libm (not f64::sqrt) so this compiles under no_std; sqrt is IEEE correctly-rounded so the
    // result matches the C++ std::sqrt used as the differential oracle.
    libm::sqrt(x * x + y * y + z * z)
}

enum StepKind {
    Inversion,
    NotInversion,
    /// A zero-length step (repeated positions) has no direction; treated as a reset, matching the
    /// C++ guard against `NaN` from normalizing a zero vector.
    Reset,
}

/// Classify one motion step (defined by three consecutive positions) as an inversion or not.
fn classify_step(prev_prev: [f64; 3], prev: [f64; 3], current: [f64; 3]) -> StepKind {
    let [ppx, ppy, ppz] = prev_prev;
    let [px, py, pz] = prev;
    let [cx, cy, cz] = current;
    let current_step = [cx - px, cy - py, cz - pz];
    let prev_step = [px - ppx, py - ppy, pz - ppz];

    let cur_norm = norm3(&current_step);
    let prev_norm = norm3(&prev_step);
    // norm is non-negative, so `<= 0.0` is exactly the `== 0.0` guard from C++.
    if cur_norm <= 0.0 || prev_norm <= 0.0 {
        return StepKind::Reset;
    }

    let [csx, csy, csz] = current_step;
    let [psx, psy, psz] = prev_step;
    let cosine_value = (csx / cur_norm) * (psx / prev_norm)
        + (csy / cur_norm) * (psy / prev_norm)
        + (csz / cur_norm) * (psz / prev_norm);

    if cosine_value < INVERSION_VECTOR_THRESHOLD {
        StepKind::Inversion
    } else {
        StepKind::NotInversion
    }
}

/// Run the consecutive-inversion counter over a stream of classified steps.
fn run_oscillation(steps: impl Iterator<Item = StepKind>) -> i32 {
    let mut oscillation_cnt: i32 = 0;
    let mut max_oscillation_cnt: i32 = 0;
    for kind in steps {
        match kind {
            StepKind::Inversion => oscillation_cnt = oscillation_cnt.saturating_add(1),
            StepKind::NotInversion | StepKind::Reset => oscillation_cnt = 0,
        }
        max_oscillation_cnt = max_oscillation_cnt.max(oscillation_cnt);
    }
    max_oscillation_cnt
}

/// Count the maximum number of consecutive direction inversions ("oscillations") in a
/// sequence of positions. Pure, `no_std`-friendly path (used by tests and the Track B engine).
#[must_use]
pub fn count_oscillation(positions: &[[f64; 3]]) -> i32 {
    run_oscillation(positions.windows(3).filter_map(|w| match w {
        [a, b, c] => Some(classify_step(*a, *b, *c)),
        _ => None,
    }))
}

/// Zero-copy ROS path: iterate `geometry_msgs::Pose` in place (no flattening / allocation),
/// reading only `position.{x,y,z}`. `windows(3)` over `&[Pose]` is a view, not a copy.
#[cfg(feature = "ros")]
#[must_use]
pub fn count_oscillation_poses(poses: &[crate::ros_msgs::geometry_msgs__msg__Pose]) -> i32 {
    fn position(pose: &crate::ros_msgs::geometry_msgs__msg__Pose) -> [f64; 3] {
        [pose.position.x, pose.position.y, pose.position.z]
    }
    run_oscillation(poses.windows(3).filter_map(|w| match w {
        [a, b, c] => Some(classify_step(position(a), position(b), position(c))),
        _ => None,
    }))
}

#[cfg(test)]
#[allow(clippy::indexing_slicing, clippy::arithmetic_side_effects)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    // 90-degree rotation about z swaps the diagonal x/y variances; other entries untouched.
    #[test]
    fn rotate_covariance_swaps_diagonal_under_quarter_turn() {
        let mut src = [0.0_f64; 36];
        src[0] = 4.0; // var x
        src[7] = 9.0; // var y
        src[14] = 16.0; // var z
        src[21] = 42.0; // sentinel in an angular entry

        // 90 deg CCW about z, row-major: [[0,-1,0],[1,0,0],[0,0,1]].
        let rot = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let out = rotate_covariance(&src, &rot);

        assert!(close(out[0], 9.0));
        assert!(close(out[7], 4.0));
        assert!(close(out[14], 16.0));
        assert!(close(out[1], 0.0));
        assert!(close(out[6], 0.0));
        assert!(close(out[21], 42.0));
    }

    #[test]
    fn rotate_covariance_identity_is_no_op() {
        let mut src = [0.0_f64; 36];
        let mut value = 1.0_f64;
        for slot in &mut src {
            *slot = value;
            value += 1.0;
        }
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let out = rotate_covariance(&src, &identity);
        for i in 0..36 {
            assert!(close(out[i], src[i]), "index {i}");
        }
    }

    fn xyz(x: f64) -> [f64; 3] {
        [x, 0.0, 0.0]
    }

    #[test]
    fn count_oscillation_counts_consecutive_inversions() {
        let poses: Vec<[f64; 3]> = [0.0, 1.0, 0.0, 1.0, 0.0].iter().map(|&x| xyz(x)).collect();
        assert_eq!(count_oscillation(&poses), 3);
    }

    #[test]
    fn count_oscillation_straight_line_is_zero() {
        let poses: Vec<[f64; 3]> = [0.0, 1.0, 2.0, 3.0, 4.0].iter().map(|&x| xyz(x)).collect();
        assert_eq!(count_oscillation(&poses), 0);
    }

    #[test]
    fn count_oscillation_resets_after_non_inversion() {
        let poses: Vec<[f64; 3]> = [0.0, 1.0, 0.0, 1.0, 2.0, 1.0]
            .iter()
            .map(|&x| xyz(x))
            .collect();
        assert_eq!(count_oscillation(&poses), 2);
    }

    #[test]
    fn count_oscillation_too_few_poses_is_zero() {
        let poses = vec![xyz(0.0), xyz(1.0)];
        assert_eq!(count_oscillation(&poses), 0);
    }

    #[test]
    fn count_oscillation_zero_length_steps_are_not_oscillations() {
        let poses = vec![[1.0, 2.0, 3.0]; 5];
        assert_eq!(count_oscillation(&poses), 0);
    }

    #[test]
    fn count_oscillation_resets_on_zero_length_step() {
        let poses: Vec<[f64; 3]> = [0.0, 1.0, 0.0, 0.0, 1.0].iter().map(|&x| xyz(x)).collect();
        assert_eq!(count_oscillation(&poses), 1);
    }

    // The zero-copy Pose path must agree with the pure path on the same positions.
    #[cfg(feature = "ros")]
    #[test]
    fn count_oscillation_poses_matches_pure_path() {
        let pose = |x: f64| {
            let mut p = crate::ros_msgs::geometry_msgs__msg__Pose::default();
            p.position.x = x;
            p
        };
        for xs in [
            vec![0.0, 1.0, 0.0, 1.0, 0.0],
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 1.0, 0.0, 1.0, 2.0, 1.0],
        ] {
            let positions: Vec<[f64; 3]> = xs.iter().map(|&x| xyz(x)).collect();
            let poses: Vec<_> = xs.iter().map(|&x| pose(x)).collect();
            assert_eq!(
                count_oscillation_poses(&poses),
                count_oscillation(&positions)
            );
        }
    }
}
