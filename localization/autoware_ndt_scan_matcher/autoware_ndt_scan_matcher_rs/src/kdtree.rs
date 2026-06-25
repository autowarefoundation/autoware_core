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

//! Static 3-D kd-tree with radius search, over voxel centroids. Replaces `pcl::KdTreeFLANN` for the
//! NDT voxel-grid map (E3). `no_std` + `alloc`; no external dependency.

// Numeric/index kernel: bounded recursion depth (~log2 N) and indexing into fixed `[f32; 3]` /
// the internal node array; distances are computed in f64 via `f64::from` (no lossy casts).
#![allow(clippy::arithmetic_side_effects, clippy::indexing_slicing)]

use alloc::vec::Vec;

struct Node {
    point_idx: usize,
    axis: usize,
    left: Option<usize>,
    right: Option<usize>,
}

/// Immutable 3-D kd-tree built once from a set of points.
pub struct KdTree {
    points: Vec<[f32; 3]>,
    nodes: Vec<Node>,
    root: Option<usize>,
}

fn dist_sq(a: &[f32; 3], b: &[f32; 3]) -> f64 {
    let dx = f64::from(a[0]) - f64::from(b[0]);
    let dy = f64::from(a[1]) - f64::from(b[1]);
    let dz = f64::from(a[2]) - f64::from(b[2]);
    (dx * dx) + (dy * dy) + (dz * dz)
}

impl KdTree {
    /// Build a balanced kd-tree from `points` (copied in).
    #[must_use]
    pub fn build(points: &[[f32; 3]]) -> Self {
        let pts = points.to_vec();
        let mut idx: Vec<usize> = (0..pts.len()).collect();
        let mut nodes: Vec<Node> = Vec::with_capacity(pts.len());
        let root = build_rec(&pts, &mut idx, 0, &mut nodes);
        Self { points: pts, nodes, root }
    }

    /// Indices of all points within Euclidean `radius` of `query`, appended to `out`.
    /// `max_nn == 0` means unlimited; otherwise the search stops once `max_nn` are collected.
    pub fn radius_search(&self, query: &[f32; 3], radius: f64, max_nn: usize, out: &mut Vec<usize>) {
        let r2 = radius * radius;
        self.search_rec(self.root, query, r2, max_nn, out);
    }

    fn search_rec(
        &self,
        node: Option<usize>,
        query: &[f32; 3],
        r2: f64,
        max_nn: usize,
        out: &mut Vec<usize>,
    ) {
        let Some(ni) = node else { return };
        if max_nn != 0 && out.len() >= max_nn {
            return;
        }
        let Some(n) = self.nodes.get(ni) else { return };
        let Some(p) = self.points.get(n.point_idx) else { return };

        if dist_sq(p, query) <= r2 {
            out.push(n.point_idx);
        }

        let diff = f64::from(query[n.axis]) - f64::from(p[n.axis]);
        let (near, far) = if diff < 0.0 { (n.left, n.right) } else { (n.right, n.left) };

        self.search_rec(near, query, r2, max_nn, out);
        // Only descend the far side if the splitting plane is within the radius.
        if (diff * diff) <= r2 {
            self.search_rec(far, query, r2, max_nn, out);
        }
    }
}

fn build_rec(
    pts: &[[f32; 3]],
    idx: &mut [usize],
    depth: usize,
    nodes: &mut Vec<Node>,
) -> Option<usize> {
    if idx.is_empty() {
        return None;
    }
    let axis = depth % 3;
    idx.sort_unstable_by(|&a, &b| pts[a][axis].total_cmp(&pts[b][axis]));
    let mid = idx.len() / 2;
    let point_idx = idx[mid];

    let (left_slice, rest) = idx.split_at_mut(mid);
    // `rest[0]` is the median; recurse on the elements after it.
    let right_slice = &mut rest[1..];
    let left = build_rec(pts, left_slice, depth + 1, nodes);
    let right = build_rec(pts, right_slice, depth + 1, nodes);

    let id = nodes.len();
    nodes.push(Node { point_idx, axis, left, right });
    Some(id)
}

#[cfg(test)]
#[allow(
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::unreadable_literal
)]
mod tests {
    use super::*;

    // Tiny deterministic LCG so the test needs no rand dependency.
    struct Lcg(u64);
    impl Lcg {
        fn next_f32(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // top 24 bits -> [0,1), then scale to [-5, 5)
            let bits = (self.0 >> 40) as u32;
            ((bits as f32 / (1u32 << 24) as f32) * 10.0) - 5.0
        }
    }

    fn brute(points: &[[f32; 3]], q: &[f32; 3], radius: f64) -> Vec<usize> {
        let r2 = radius * radius;
        let mut v: Vec<usize> = (0..points.len()).filter(|&i| dist_sq(&points[i], q) <= r2).collect();
        v.sort_unstable();
        v
    }

    #[test]
    fn radius_search_matches_brute_force() {
        let mut rng = Lcg(0x1234_5678);
        let pts: Vec<[f32; 3]> =
            (0..500).map(|_| [rng.next_f32(), rng.next_f32(), rng.next_f32()]).collect();
        let tree = KdTree::build(&pts);

        for _ in 0..50 {
            let q = [rng.next_f32(), rng.next_f32(), rng.next_f32()];
            let radius = 1.3_f64;
            let mut got = Vec::new();
            tree.radius_search(&q, radius, 0, &mut got);
            got.sort_unstable();
            assert_eq!(got, brute(&pts, &q, radius));
        }
    }

    #[test]
    fn empty_tree_returns_nothing() {
        let tree = KdTree::build(&[]);
        let mut out = Vec::new();
        tree.radius_search(&[0.0, 0.0, 0.0], 1.0, 0, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn max_nn_caps_results() {
        let pts: Vec<[f32; 3]> = (0..10).map(|i| [i as f32 * 0.01, 0.0, 0.0]).collect();
        let tree = KdTree::build(&pts);
        let mut out = Vec::new();
        tree.radius_search(&[0.05, 0.0, 0.0], 1.0, 3, &mut out);
        assert_eq!(out.len(), 3);
    }
}
