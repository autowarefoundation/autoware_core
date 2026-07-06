// Copyright 2026 Autoware Foundation
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

//! Pure Rust TPE core for the align-service search port.
//!
//! This mirrors the semantic algorithm of
//! `autoware::localization_util::TreeStructuredParzenEstimator`: five prior dimensions
//! (`x, y, z, roll, pitch`) plus uniformly sampled yaw, startup sampling from the prior, and then
//! expected-improvement candidate selection using above/below Gaussian KDEs. It intentionally does
//! not try to reproduce libstdc++'s exact `std::normal_distribution` samples.

use alloc::vec::Vec;

const INPUT_DIMENSION: usize = 6;
const PRIOR_DIMENSION: usize = 5;
const MAX_GOOD_NUM: usize = 10;
const N_EI_CANDIDATES: usize = 100;
const DEFAULT_SEED: u64 = 0;
const PI: f64 = core::f64::consts::PI;
const TWO_PI: f64 = core::f64::consts::TAU;
const UNIT_U32_SCALE: f64 = 4_294_967_296.0;
const BASE_STDDEV: Input = [
    0.25,
    0.25,
    0.25,
    1.0 / 180.0 * PI,
    1.0 / 180.0 * PI,
    2.5 / 180.0 * PI,
];

pub type Input = [f64; INPUT_DIMENSION];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    Minimize,
    Maximize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error {
    InvalidMeanLen,
    InvalidStddevLen,
    InvalidStartupTrials,
    InvalidMean,
    InvalidStddev,
    InvalidTrialInput,
    InvalidTrialScore,
    InvalidKdeState,
    ConversionFailed,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trial {
    pub input: Input,
    pub score: f64,
}

#[derive(Clone, Copy, Debug)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "SplitMix64 is defined by explicit wrapping integer arithmetic and shifts"
    )]
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "floating-point scaling from u32 to [0, 1) cannot integer-overflow"
    )]
    fn next_unit_f64(&mut self) -> Result<f64, Error> {
        let upper = u32::try_from(self.next_u64() >> 32).map_err(|_| Error::ConversionFailed)?;
        Ok(f64::from(upper) / UNIT_U32_SCALE)
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "Box-Muller transform is floating-point math"
    )]
    fn normal_standard(&mut self) -> Result<f64, Error> {
        let mut u1 = self.next_unit_f64()?;
        while u1 <= f64::MIN_POSITIVE {
            u1 = self.next_unit_f64()?;
        }
        let u2 = self.next_unit_f64()?;
        let radius = libm::sqrt(-2.0 * libm::log(u1));
        let theta = TWO_PI * u2;
        Ok(radius * libm::cos(theta))
    }
}

#[derive(Clone, Debug)]
pub struct TreeStructuredParzenEstimator {
    direction: Direction,
    n_startup_trials: usize,
    sample_mean: [f64; PRIOR_DIMENSION],
    sample_stddev: [f64; PRIOR_DIMENSION],
    trials: Vec<Trial>,
    above_num: usize,
    rng: SplitMix64,
}

impl TreeStructuredParzenEstimator {
    /// Construct a TPE with the default deterministic Rust seed.
    ///
    /// # Errors
    /// Returns an error when the prior shape is invalid, startup trial count is negative, or any
    /// prior mean/stddev value is non-finite or unusable.
    pub fn new(
        direction: Direction,
        n_startup_trials: i64,
        sample_mean: &[f64],
        sample_stddev: &[f64],
    ) -> Result<Self, Error> {
        Self::with_seed(
            direction,
            n_startup_trials,
            sample_mean,
            sample_stddev,
            DEFAULT_SEED,
        )
    }

    /// Construct a TPE with an explicit deterministic Rust seed.
    ///
    /// # Errors
    /// Returns an error when the prior shape is invalid, startup trial count is negative, or any
    /// prior mean/stddev value is non-finite or unusable.
    pub fn with_seed(
        direction: Direction,
        n_startup_trials: i64,
        sample_mean: &[f64],
        sample_stddev: &[f64],
        seed: u64,
    ) -> Result<Self, Error> {
        if n_startup_trials < 0 {
            return Err(Error::InvalidStartupTrials);
        }
        let sample_mean = prior_array(sample_mean, Error::InvalidMeanLen)?;
        let sample_stddev = prior_array(sample_stddev, Error::InvalidStddevLen)?;
        if !sample_mean.iter().copied().all(f64::is_finite) {
            return Err(Error::InvalidMean);
        }
        if !sample_stddev.iter().copied().all(valid_positive_finite) {
            return Err(Error::InvalidStddev);
        }
        if !BASE_STDDEV.iter().copied().all(valid_positive_finite) {
            return Err(Error::InvalidStddev);
        }
        Ok(Self {
            direction,
            n_startup_trials: usize::try_from(n_startup_trials)
                .map_err(|_| Error::InvalidStartupTrials)?,
            sample_mean,
            sample_stddev,
            trials: Vec::new(),
            above_num: 0,
            rng: SplitMix64::new(seed),
        })
    }

    #[must_use]
    pub fn trials_len(&self) -> usize {
        self.trials.len()
    }

    #[must_use]
    pub fn above_num(&self) -> usize {
        self.above_num
    }

    /// Add one evaluated trial and update the sorted good/bad KDE partition.
    ///
    /// # Errors
    /// Returns an error if the trial input or score contains a non-finite value.
    pub fn add_trial(&mut self, trial: Trial) -> Result<(), Error> {
        if !trial.input.iter().copied().all(f64::is_finite) {
            return Err(Error::InvalidTrialInput);
        }
        if !trial.score.is_finite() {
            return Err(Error::InvalidTrialScore);
        }
        self.trials.push(trial);
        match self.direction {
            Direction::Maximize => self
                .trials
                .sort_by(|lhs, rhs| rhs.score.total_cmp(&lhs.score)),
            Direction::Minimize => self
                .trials
                .sort_by(|lhs, rhs| lhs.score.total_cmp(&rhs.score)),
        }
        self.above_num = core::cmp::min(MAX_GOOD_NUM, self.trials.len() / 10);
        Ok(())
    }

    /// Generate the next candidate input.
    ///
    /// # Errors
    /// Returns an error if the internal KDE partition is invalid or deterministic sampling fails.
    pub fn get_next_input(&mut self) -> Result<Input, Error> {
        if self.trials.len() < self.n_startup_trials || self.above_num == 0 {
            return self.sample_prior();
        }

        let mut best_input = None;
        let mut best_log_likelihood_ratio = f64::NEG_INFINITY;
        for _ in 0..N_EI_CANDIDATES {
            let input = self.sample_prior()?;
            let log_likelihood_ratio = self.compute_log_likelihood_ratio(&input)?;
            if log_likelihood_ratio > best_log_likelihood_ratio {
                best_log_likelihood_ratio = log_likelihood_ratio;
                best_input = Some(input);
            }
        }
        best_input.ok_or(Error::InvalidKdeState)
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "sampling priors is floating-point math"
    )]
    fn sample_prior(&mut self) -> Result<Input, Error> {
        let [mx, my, mz, mroll, mpitch] = self.sample_mean;
        let [sx, sy, sz, sroll, spitch] = self.sample_stddev;
        Ok([
            mx + (sx * self.rng.normal_standard()?),
            my + (sy * self.rng.normal_standard()?),
            mz + (sz * self.rng.normal_standard()?),
            mroll + (sroll * self.rng.normal_standard()?),
            mpitch + (spitch * self.rng.normal_standard()?),
            -PI + (TWO_PI * self.rng.next_unit_f64()?),
        ])
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "KDE likelihood ratio is floating-point math"
    )]
    fn compute_log_likelihood_ratio(&self, input: &Input) -> Result<f64, Error> {
        let below_num = self
            .trials
            .len()
            .checked_sub(self.above_num)
            .ok_or(Error::InvalidKdeState)?;
        if self.above_num == 0 || below_num == 0 {
            return Err(Error::InvalidKdeState);
        }

        let above_weight_log = libm::log(1.0 / usize_to_f64(self.above_num)?);
        let below_weight_log = libm::log(1.0 / usize_to_f64(below_num)?);
        let mut above_logs = Vec::new();
        let mut below_logs = Vec::new();
        for (i, trial) in self.trials.iter().enumerate() {
            let log_p = log_gaussian_pdf(input, &trial.input, &BASE_STDDEV)?;
            if i < self.above_num {
                above_logs.push(log_p + above_weight_log);
            } else {
                below_logs.push(log_p + below_weight_log);
            }
        }

        let above = log_sum_exp(&above_logs)?;
        let below = log_sum_exp(&below_logs)?;
        Ok(above - (below * 5.0))
    }
}

fn prior_array(input: &[f64], len_error: Error) -> Result<[f64; PRIOR_DIMENSION], Error> {
    let got: [f64; PRIOR_DIMENSION] = input.try_into().map_err(|_| len_error)?;
    Ok(got)
}

fn valid_positive_finite(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn usize_to_f64(value: usize) -> Result<f64, Error> {
    let value = u32::try_from(value).map_err(|_| Error::ConversionFailed)?;
    Ok(f64::from(value))
}

#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "log-sum-exp is floating-point math"
)]
fn log_sum_exp(values: &[f64]) -> Result<f64, Error> {
    if values.is_empty() {
        return Err(Error::InvalidKdeState);
    }
    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let sum = values
        .iter()
        .copied()
        .map(|value| libm::exp(value - max))
        .sum::<f64>();
    Ok(max + libm::log(sum))
}

#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "Gaussian PDF is floating-point math"
)]
fn log_gaussian_pdf(input: &Input, mu: &Input, sigma: &Input) -> Result<f64, Error> {
    let [ix, iy, _iz, _iroll, _ipitch, iyaw] = *input;
    let [mx, my, _mz, _mroll, _mpitch, myaw] = *mu;
    let [sx, sy, _sz, _sroll, _spitch, syaw] = *sigma;
    if !valid_positive_finite(sx) || !valid_positive_finite(sy) || !valid_positive_finite(syaw) {
        return Err(Error::InvalidStddev);
    }
    Ok(log_gaussian_pdf_1d(ix - mx, sx)
        + log_gaussian_pdf_1d(iy - my, sy)
        + log_gaussian_pdf_1d(normalize_radian(iyaw - myaw), syaw))
}

#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "Gaussian PDF is floating-point math"
)]
fn log_gaussian_pdf_1d(diff: f64, sigma: f64) -> f64 {
    let log_2pi = libm::log(TWO_PI);
    -0.5 * log_2pi - libm::log(sigma) - ((diff * diff) / (2.0 * sigma * sigma))
}

#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "angle normalization is floating-point math"
)]
fn normalize_radian(mut rad: f64) -> f64 {
    while rad >= PI {
        rad -= TWO_PI;
    }
    while rad < -PI {
        rad += TWO_PI;
    }
    rad
}

#[cfg(feature = "std")]
pub const AW_TPE_STATUS_OK: i32 = 0;
#[cfg(feature = "std")]
pub const AW_TPE_STATUS_NULL_POINTER: i32 = 1;
#[cfg(feature = "std")]
pub const AW_TPE_STATUS_INVALID_LENGTH: i32 = 2;
#[cfg(feature = "std")]
pub const AW_TPE_STATUS_INVALID_DIRECTION: i32 = 3;
#[cfg(feature = "std")]
pub const AW_TPE_STATUS_INVALID_INPUT: i32 = 4;
#[cfg(feature = "std")]
pub const AW_TPE_STATUS_INTERNAL_ERROR: i32 = 5;
#[cfg(feature = "std")]
pub const AW_TPE_DIRECTION_MINIMIZE: i32 = 0;
#[cfg(feature = "std")]
pub const AW_TPE_DIRECTION_MAXIMIZE: i32 = 1;

#[cfg(feature = "std")]
pub struct AwTpe {
    inner: TreeStructuredParzenEstimator,
}

#[cfg(feature = "std")]
fn direction_from_ffi(direction: i32) -> Result<Direction, i32> {
    match direction {
        AW_TPE_DIRECTION_MINIMIZE => Ok(Direction::Minimize),
        AW_TPE_DIRECTION_MAXIMIZE => Ok(Direction::Maximize),
        _ => Err(AW_TPE_STATUS_INVALID_DIRECTION),
    }
}

#[cfg(feature = "std")]
fn status_from_error(error: Error) -> i32 {
    match error {
        Error::InvalidMeanLen | Error::InvalidStddevLen => AW_TPE_STATUS_INVALID_LENGTH,
        Error::InvalidStartupTrials
        | Error::InvalidMean
        | Error::InvalidStddev
        | Error::InvalidTrialInput
        | Error::InvalidTrialScore => AW_TPE_STATUS_INVALID_INPUT,
        Error::InvalidKdeState | Error::ConversionFailed => AW_TPE_STATUS_INTERNAL_ERROR,
    }
}

#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI helper; converts validated pointer and length to a slice"
)]
fn checked_slice<'a>(ptr: *const f64, len: usize, expected_len: usize) -> Result<&'a [f64], i32> {
    if len != expected_len {
        return Err(AW_TPE_STATUS_INVALID_LENGTH);
    }
    if ptr.is_null() {
        return Err(AW_TPE_STATUS_NULL_POINTER);
    }
    // SAFETY: pointer is non-null and caller promises `len` readable f64 values.
    Ok(unsafe { core::slice::from_raw_parts(ptr, len) })
}

#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI helper; converts validated pointer and length to a mutable slice"
)]
fn checked_output_slice<'a>(
    ptr: *mut f64,
    len: usize,
    expected_len: usize,
) -> Result<&'a mut [f64], i32> {
    if len != expected_len {
        return Err(AW_TPE_STATUS_INVALID_LENGTH);
    }
    if ptr.is_null() {
        return Err(AW_TPE_STATUS_NULL_POINTER);
    }
    // SAFETY: pointer is non-null and caller promises `len` writable f64 values.
    Ok(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
}

/// Construct an owned Rust TPE handle. Returns null on invalid input.
///
/// # Safety
/// `sample_mean` and `sample_stddev` must each point to five readable `f64` values. Rust copies the
/// slices and does not retain the pointers. Free the returned handle with
/// [`autoware_ndt_scan_matcher_rs_tpe_free`].
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned mean/stddev buffers after null/length checks"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_new(
    direction: i32,
    n_startup_trials: i64,
    sample_mean: *const f64,
    sample_mean_len: usize,
    sample_stddev: *const f64,
    sample_stddev_len: usize,
    seed: u64,
) -> *mut AwTpe {
    let Ok(direction) = direction_from_ffi(direction) else {
        return core::ptr::null_mut();
    };
    let Ok(mean) = checked_slice(sample_mean, sample_mean_len, PRIOR_DIMENSION) else {
        return core::ptr::null_mut();
    };
    let Ok(stddev) = checked_slice(sample_stddev, sample_stddev_len, PRIOR_DIMENSION) else {
        return core::ptr::null_mut();
    };
    match TreeStructuredParzenEstimator::with_seed(direction, n_startup_trials, mean, stddev, seed)
    {
        Ok(inner) => Box::into_raw(Box::new(AwTpe { inner })),
        Err(_) => core::ptr::null_mut(),
    }
}

/// Free an owned TPE handle returned by [`autoware_ndt_scan_matcher_rs_tpe_new`].
///
/// # Safety
/// `handle` must be null or a live handle returned by this crate. Passing the same handle twice is
/// invalid.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reclaims Box-owned opaque handle"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_free(handle: *mut AwTpe) {
    if !handle.is_null() {
        // SAFETY: non-null handle is owned by the caller and was allocated by `tpe_new`.
        unsafe {
            drop(Box::from_raw(handle));
        }
    }
}

/// Generate one TPE input into `out_input`.
///
/// # Safety
/// `handle` must be a live TPE handle, and `out_input` must point to six writable `f64` slots.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned output buffer after null/length checks"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_get_next_input(
    handle: *mut AwTpe,
    out_input: *mut f64,
    out_len: usize,
) -> i32 {
    if handle.is_null() {
        return AW_TPE_STATUS_NULL_POINTER;
    }
    let Ok(out) = checked_output_slice(out_input, out_len, INPUT_DIMENSION) else {
        return if out_len == INPUT_DIMENSION {
            AW_TPE_STATUS_NULL_POINTER
        } else {
            AW_TPE_STATUS_INVALID_LENGTH
        };
    };
    // SAFETY: handle is non-null and caller promises it is a live unique mutable handle.
    let tpe = unsafe { &mut *handle };
    match tpe.inner.get_next_input() {
        Ok(input) => {
            for (dst, src) in out.iter_mut().zip(input.iter().copied()) {
                *dst = src;
            }
            AW_TPE_STATUS_OK
        }
        Err(error) => status_from_error(error),
    }
}

/// Add one evaluated trial to the TPE handle.
///
/// # Safety
/// `handle` must be a live TPE handle, and `input` must point to six readable `f64` values.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned input buffer after null/length checks"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_add_trial(
    handle: *mut AwTpe,
    input: *const f64,
    input_len: usize,
    score: f64,
) -> i32 {
    if handle.is_null() {
        return AW_TPE_STATUS_NULL_POINTER;
    }
    let Ok(input_slice) = checked_slice(input, input_len, INPUT_DIMENSION) else {
        return if input_len == INPUT_DIMENSION {
            AW_TPE_STATUS_NULL_POINTER
        } else {
            AW_TPE_STATUS_INVALID_LENGTH
        };
    };
    let Ok(input) = <[f64; INPUT_DIMENSION]>::try_from(input_slice) else {
        return AW_TPE_STATUS_INVALID_LENGTH;
    };
    // SAFETY: handle is non-null and caller promises it is a live unique mutable handle.
    let tpe = unsafe { &mut *handle };
    match tpe.inner.add_trial(Trial { input, score }) {
        Ok(()) => AW_TPE_STATUS_OK,
        Err(error) => status_from_error(error),
    }
}

/// Return the number of trials currently stored in the TPE handle. Null returns zero.
///
/// # Safety
/// `handle` must be null or a live TPE handle.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads opaque handle after null check"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_trials_len(
    handle: *const AwTpe,
) -> usize {
    if handle.is_null() {
        return 0;
    }
    // SAFETY: handle is non-null and caller promises it is live for the duration of the call.
    let tpe = unsafe { &*handle };
    tpe.inner.trials_len()
}

/// Return the current number of good trials in the TPE KDE partition. Null returns zero.
///
/// # Safety
/// `handle` must be null or a live TPE handle.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads opaque handle after null check"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_tpe_above_num(handle: *const AwTpe) -> usize {
    if handle.is_null() {
        return 0;
    }
    // SAFETY: handle is non-null and caller promises it is live for the duration of the call.
    let tpe = unsafe { &*handle };
    tpe.inner.above_num()
}

#[cfg(test)]
#[allow(
    clippy::allow_attributes,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::unwrap_used,
    reason = "test code uses direct assertions and fixed-size fixture indexing"
)]
mod tests {
    use super::*;

    fn sample_mean() -> [f64; 5] {
        [0.0, 0.0, 0.0, 0.0, 0.0]
    }

    fn sample_stddev() -> [f64; 5] {
        [1.0, 1.0, 0.1, 0.1, 0.1]
    }

    fn finite_input(input: &Input) {
        assert!(input.iter().all(|v| v.is_finite()));
        assert!(input[5] >= -PI);
        assert!(input[5] < PI);
    }

    fn objective(input: &Input) -> f64 {
        -((input[0] * input[0]) + (input[1] * input[1]))
    }

    #[test]
    fn constructor_validates_lengths_and_values() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        assert!(
            TreeStructuredParzenEstimator::new(Direction::Maximize, 10, &mean, &stddev).is_ok()
        );
        assert_eq!(
            TreeStructuredParzenEstimator::new(Direction::Maximize, 10, &mean[..4], &stddev)
                .unwrap_err(),
            Error::InvalidMeanLen
        );
        assert_eq!(
            TreeStructuredParzenEstimator::new(Direction::Maximize, 10, &mean, &stddev[..4])
                .unwrap_err(),
            Error::InvalidStddevLen
        );
        let mut bad_stddev = stddev;
        bad_stddev[0] = 0.0;
        assert_eq!(
            TreeStructuredParzenEstimator::new(Direction::Maximize, 10, &mean, &bad_stddev)
                .unwrap_err(),
            Error::InvalidStddev
        );
        assert_eq!(
            TreeStructuredParzenEstimator::new(Direction::Maximize, -1, &mean, &stddev)
                .unwrap_err(),
            Error::InvalidStartupTrials
        );
    }

    #[test]
    fn startup_phase_samples_finite_six_dimensional_inputs() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        let mut tpe =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 10, &mean, &stddev, 7)
                .unwrap();
        finite_input(&tpe.get_next_input().unwrap());
        for i in 0..5 {
            tpe.add_trial(Trial {
                input: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                score: f64::from(i),
            })
            .unwrap();
        }
        finite_input(&tpe.get_next_input().unwrap());
        assert_eq!(tpe.above_num(), 0);
    }

    #[test]
    fn add_trial_sorts_and_updates_above_num() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        let mut maximize =
            TreeStructuredParzenEstimator::new(Direction::Maximize, 0, &mean, &stddev).unwrap();
        let mut minimize =
            TreeStructuredParzenEstimator::new(Direction::Minimize, 0, &mean, &stddev).unwrap();
        for score in [10.0, 20.0, 5.0, 15.0, 30.0, 25.0, 0.0, 1.0, 2.0, 3.0] {
            let input = [score, 0.0, 0.0, 0.0, 0.0, 0.0];
            maximize.add_trial(Trial { input, score }).unwrap();
            minimize.add_trial(Trial { input, score }).unwrap();
        }
        assert_eq!(maximize.above_num(), 1);
        assert_eq!(minimize.above_num(), 1);
        assert_eq!(maximize.trials[0].score, 30.0);
        assert_eq!(minimize.trials[0].score, 0.0);
    }

    #[test]
    fn optimization_phase_returns_finite_candidate() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        let mut tpe =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 5, &mean, &stddev, 11)
                .unwrap();
        for i in 0..10 {
            let x = f64::from(i) * 0.1;
            let input = [x, -x, 0.0, 0.0, 0.0, 0.0];
            tpe.add_trial(Trial {
                input,
                score: objective(&input),
            })
            .unwrap();
        }
        finite_input(&tpe.get_next_input().unwrap());
    }

    #[test]
    fn angle_wrapping_makes_opposite_pi_neighbors_close() {
        let a = [0.0, 0.0, 0.0, 0.0, 0.0, PI - 0.1];
        let b = [0.0, 0.0, 0.0, 0.0, 0.0, -PI + 0.1];
        let log_p = log_gaussian_pdf(&a, &b, &BASE_STDDEV).unwrap();
        let far = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let far_log_p = log_gaussian_pdf(&far, &b, &BASE_STDDEV).unwrap();
        assert!(log_p > far_log_p);
    }

    #[test]
    fn fixed_seed_reproduces_sequence() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        let mut a =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 5, &mean, &stddev, 123)
                .unwrap();
        let mut b =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 5, &mean, &stddev, 123)
                .unwrap();
        for _ in 0..20 {
            assert_eq!(a.get_next_input().unwrap(), b.get_next_input().unwrap());
        }
    }

    #[test]
    fn tpe_improves_over_prior_only_on_quadratic() {
        let mean = [1.0, 1.0, 0.0, 0.0, 0.0];
        let stddev = [1.0, 1.0, 0.1, 0.1, 0.1];
        let mut random =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 120, &mean, &stddev, 99)
                .unwrap();
        let mut tpe =
            TreeStructuredParzenEstimator::with_seed(Direction::Maximize, 40, &mean, &stddev, 99)
                .unwrap();
        let mut random_best = f64::NEG_INFINITY;
        let mut tpe_best = f64::NEG_INFINITY;
        for _ in 0..120 {
            let random_input = random.get_next_input().unwrap();
            let random_score = objective(&random_input);
            random
                .add_trial(Trial {
                    input: random_input,
                    score: random_score,
                })
                .unwrap();
            random_best = random_best.max(random_score);

            let tpe_input = tpe.get_next_input().unwrap();
            let tpe_score = objective(&tpe_input);
            tpe.add_trial(Trial {
                input: tpe_input,
                score: tpe_score,
            })
            .unwrap();
            tpe_best = tpe_best.max(tpe_score);
        }
        assert!(tpe_best >= random_best);
    }

    #[cfg(feature = "std")]
    #[allow(unsafe_code, reason = "test exercises the C ABI entry points directly")]
    #[test]
    fn ffi_rejects_nulls_and_invalid_lengths() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        unsafe {
            assert!(
                autoware_ndt_scan_matcher_rs_tpe_new(
                    AW_TPE_DIRECTION_MAXIMIZE,
                    10,
                    mean.as_ptr(),
                    4,
                    stddev.as_ptr(),
                    stddev.len(),
                    0,
                )
                .is_null()
            );
            assert!(
                autoware_ndt_scan_matcher_rs_tpe_new(
                    99,
                    10,
                    mean.as_ptr(),
                    mean.len(),
                    stddev.as_ptr(),
                    stddev.len(),
                    0,
                )
                .is_null()
            );
            let handle = autoware_ndt_scan_matcher_rs_tpe_new(
                AW_TPE_DIRECTION_MAXIMIZE,
                10,
                mean.as_ptr(),
                mean.len(),
                stddev.as_ptr(),
                stddev.len(),
                0,
            );
            assert!(!handle.is_null());
            let mut out = [0.0; 6];
            assert_eq!(
                autoware_ndt_scan_matcher_rs_tpe_get_next_input(
                    core::ptr::null_mut(),
                    out.as_mut_ptr(),
                    out.len(),
                ),
                AW_TPE_STATUS_NULL_POINTER
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_tpe_get_next_input(
                    handle,
                    core::ptr::null_mut(),
                    out.len(),
                ),
                AW_TPE_STATUS_NULL_POINTER
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_tpe_get_next_input(handle, out.as_mut_ptr(), 5),
                AW_TPE_STATUS_INVALID_LENGTH
            );
            autoware_ndt_scan_matcher_rs_tpe_free(handle);
        }
    }

    #[cfg(feature = "std")]
    #[allow(unsafe_code, reason = "test exercises the C ABI entry points directly")]
    #[test]
    fn ffi_generates_inputs_and_updates_trials() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        unsafe {
            let handle = autoware_ndt_scan_matcher_rs_tpe_new(
                AW_TPE_DIRECTION_MAXIMIZE,
                5,
                mean.as_ptr(),
                mean.len(),
                stddev.as_ptr(),
                stddev.len(),
                42,
            );
            assert!(!handle.is_null());
            let mut input = [0.0; 6];
            assert_eq!(
                autoware_ndt_scan_matcher_rs_tpe_get_next_input(
                    handle,
                    input.as_mut_ptr(),
                    input.len(),
                ),
                AW_TPE_STATUS_OK
            );
            finite_input(&input);
            for i in 0..10 {
                let trial = [f64::from(i), 0.0, 0.0, 0.0, 0.0, 0.0];
                assert_eq!(
                    autoware_ndt_scan_matcher_rs_tpe_add_trial(
                        handle,
                        trial.as_ptr(),
                        trial.len(),
                        f64::from(i),
                    ),
                    AW_TPE_STATUS_OK
                );
            }
            assert_eq!(autoware_ndt_scan_matcher_rs_tpe_trials_len(handle), 10);
            assert_eq!(autoware_ndt_scan_matcher_rs_tpe_above_num(handle), 1);
            autoware_ndt_scan_matcher_rs_tpe_free(handle);
        }
    }

    #[cfg(feature = "std")]
    #[allow(unsafe_code, reason = "test exercises the C ABI entry points directly")]
    #[test]
    fn ffi_fixed_seed_reproduces_sequence() {
        let mean = sample_mean();
        let stddev = sample_stddev();
        unsafe {
            let a = autoware_ndt_scan_matcher_rs_tpe_new(
                AW_TPE_DIRECTION_MAXIMIZE,
                5,
                mean.as_ptr(),
                mean.len(),
                stddev.as_ptr(),
                stddev.len(),
                77,
            );
            let b = autoware_ndt_scan_matcher_rs_tpe_new(
                AW_TPE_DIRECTION_MAXIMIZE,
                5,
                mean.as_ptr(),
                mean.len(),
                stddev.as_ptr(),
                stddev.len(),
                77,
            );
            assert!(!a.is_null());
            assert!(!b.is_null());
            for _ in 0..8 {
                let mut ia = [0.0; 6];
                let mut ib = [0.0; 6];
                assert_eq!(
                    autoware_ndt_scan_matcher_rs_tpe_get_next_input(a, ia.as_mut_ptr(), ia.len()),
                    AW_TPE_STATUS_OK
                );
                assert_eq!(
                    autoware_ndt_scan_matcher_rs_tpe_get_next_input(b, ib.as_mut_ptr(), ib.len()),
                    AW_TPE_STATUS_OK
                );
                assert_eq!(ia, ib);
            }
            autoware_ndt_scan_matcher_rs_tpe_free(a);
            autoware_ndt_scan_matcher_rs_tpe_free(b);
        }
    }
}
