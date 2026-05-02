// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory/interpolator/pchip.hpp"

#include "autoware/trajectory/detail/helpers.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <utility>
#include <vector>
namespace autoware::experimental::trajectory::interpolator
{

namespace
{

[[nodiscard]] bool almost_zero(const double x, const double eps)
{
  return std::abs(x) <= eps;
}

}  // namespace

double Pchip::compute_endpoint_derivative(
  const double h0, const double h1, const double delta0, const double delta1)
{
  const double d = ((2.0 * h0 + h1) * delta0 - h0 * delta1) / (h0 + h1);

  if (d == 0.0) {
    return 0.0;
  }

  if ((d > 0.0 && delta0 <= 0.0) || (d < 0.0 && delta0 >= 0.0)) {
    return 0.0;
  }

  if ((delta0 > 0.0 && delta1 < 0.0) || (delta0 < 0.0 && delta1 > 0.0)) {
    const double limit = 3.0 * delta0;
    if (std::abs(d) > std::abs(limit)) {
      return limit;
    }
  }

  return d;
}

void Pchip::compute_parameters(
  const Eigen::Ref<const Eigen::VectorXd> & bases, const Eigen::Ref<const Eigen::VectorXd> & values)
{
  const int32_t n_points = static_cast<int32_t>(bases.size());
  const int32_t n_intervals = n_points - 1;

  a_.resize(n_intervals);
  b_.resize(n_intervals);
  c_.resize(n_intervals);
  d_.resize(n_intervals);
  h_.resize(n_intervals);
  m_.resize(n_points);

  Eigen::VectorXd delta(n_intervals);

  for (int32_t i = 0; i < n_intervals; ++i) {
    h_(i) = bases(i + 1) - bases(i);
    delta(i) = (values(i + 1) - values(i)) / h_(i);
  }

  if (n_points == 2) {
    m_(0) = delta(0);
    m_(1) = delta(0);
  } else {
    m_(0) = compute_endpoint_derivative(h_(0), h_(1), delta(0), delta(1));
    m_(n_points - 1) = compute_endpoint_derivative(
      h_(n_intervals - 1), h_(n_intervals - 2), delta(n_intervals - 1), delta(n_intervals - 2));

    for (int32_t i = 1; i < n_points - 1; ++i) {
      const double delta_prev = delta(i - 1);
      const double delta_next = delta(i);

      if (
        almost_zero(delta_prev, epsilon_) || almost_zero(delta_next, epsilon_) ||
        (delta_prev > 0.0 && delta_next < 0.0) || (delta_prev < 0.0 && delta_next > 0.0)) {
        m_(i) = 0.0;
        continue;
      }

      const double w1 = 2.0 * h_(i) + h_(i - 1);
      const double w2 = h_(i) + 2.0 * h_(i - 1);
      m_(i) = (w1 + w2) / (w1 / delta_prev + w2 / delta_next);
    }
  }

  for (int32_t i = 0; i < n_intervals; ++i) {
    const double hi = h_(i);
    const double di = delta(i);
    const double mi = m_(i);
    const double mip1 = m_(i + 1);

    a_(i) = values(i);
    b_(i) = mi;
    c_(i) = (3.0 * di - 2.0 * mi - mip1) / hi;
    d_(i) = (mi + mip1 - 2.0 * di) / (hi * hi);
  }
}

bool Pchip::build_impl(const std::vector<double> & bases, const std::vector<double> & values)
{
  auto [cleaned_bases, cleaned_values] =
    ::autoware::experimental::trajectory::detail::remove_duplicate_points(bases, values, epsilon_);

  if (cleaned_bases.size() < minimum_required_points()) {
    return false;
  }

  if (!::autoware::experimental::trajectory::detail::has_strictly_increasing_bases(
        cleaned_bases, epsilon_)) {
    return false;
  }

  this->bases_ = std::move(cleaned_bases);

  compute_parameters(
    Eigen::Map<const Eigen::VectorXd>(
      this->bases_.data(), static_cast<Eigen::Index>(this->bases_.size())),
    Eigen::Map<const Eigen::VectorXd>(
      cleaned_values.data(), static_cast<Eigen::Index>(cleaned_values.size())));

  return true;
}

bool Pchip::build_impl(const std::vector<double> & bases, std::vector<double> && values)
{
  auto [cleaned_bases, cleaned_values] =
    ::autoware::experimental::trajectory::detail::remove_duplicate_points(bases, values, epsilon_);

  if (cleaned_bases.size() < minimum_required_points()) {
    return false;
  }

  if (!::autoware::experimental::trajectory::detail::has_strictly_increasing_bases(
        cleaned_bases, epsilon_)) {
    return false;
  }

  this->bases_ = std::move(cleaned_bases);

  compute_parameters(
    Eigen::Map<const Eigen::VectorXd>(
      this->bases_.data(), static_cast<Eigen::Index>(this->bases_.size())),
    Eigen::Map<const Eigen::VectorXd>(
      cleaned_values.data(), static_cast<Eigen::Index>(cleaned_values.size())));

  return true;
}

double Pchip::compute_impl(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return a_(i) + b_(i) * dx + c_(i) * dx * dx + d_(i) * dx * dx * dx;
}

double Pchip::compute_first_derivative_impl(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return b_(i) + 2.0 * c_(i) * dx + 3.0 * d_(i) * dx * dx;
}

double Pchip::compute_second_derivative_impl(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return 2.0 * c_(i) + 6.0 * d_(i) * dx;
}

}  // namespace autoware::experimental::trajectory::interpolator
