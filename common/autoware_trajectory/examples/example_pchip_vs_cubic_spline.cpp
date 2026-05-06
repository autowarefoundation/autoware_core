// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/pchip.hpp"

#include <autoware/pyplot/pyplot.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <stdexcept>
#include <vector>

int main()
{
  try {
    pybind11::scoped_interpreter guard{};
    auto plt = autoware::pyplot::import();
    auto [fig, axes] = plt.subplots(1, 2);
    auto & ax_curve = axes[0];
    auto & ax_derivative = axes[1];

    using autoware::experimental::trajectory::interpolator::CubicSpline;
    using autoware::experimental::trajectory::interpolator::Pchip;

    // This input highlights the difference between shape-preserving interpolation
    // and a regular cubic spline around a sharp local transition.
    const std::vector<double> bases = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const std::vector<double> values = {0.0, 0.3, 0.8, 2.5, 2.7, 2.8, 2.85};

    const auto cubic_result = CubicSpline::Builder{}.set_bases(bases).set_values(values).build();
    if (!cubic_result) {
      throw std::runtime_error("Failed to build CubicSpline: " + cubic_result.error().what);
    }

    const auto pchip_result = Pchip::Builder{}.set_bases(bases).set_values(values).build();
    if (!pchip_result) {
      throw std::runtime_error("Failed to build Pchip: " + pchip_result.error().what);
    }

    const auto & cubic = cubic_result.value();
    const auto & pchip = pchip_result.value();

    const auto xs = cubic.base_arange(0.02);
    const auto cubic_values = cubic.compute(xs);
    const auto pchip_values = pchip.compute(xs);
    const auto cubic_first = cubic.compute_first_derivative(xs);
    const auto pchip_first = pchip.compute_first_derivative(xs);
    const auto cubic_second = cubic.compute_second_derivative(xs);
    const auto pchip_second = pchip.compute_second_derivative(xs);

    ax_curve.scatter(
      Args(bases, values), Kwargs("color"_a = "black", "marker"_a = "o", "label"_a = "underlying"));
    ax_curve.plot(Args(xs, cubic_values), Kwargs("color"_a = "purple", "label"_a = "cubic spline"));
    ax_curve.plot(Args(xs, pchip_values), Kwargs("color"_a = "teal", "label"_a = "pchip"));
    ax_curve.grid();
    ax_curve.legend();
    ax_curve.set_title(Args("Interpolation"));
    ax_curve.set_aspect(Args("equal"));
    ax_curve.set_xlim(Args(-0.2, 6.2));
    ax_curve.set_ylim(Args(-0.2, 3.2));

    ax_derivative.plot(
      Args(xs, cubic_first), Kwargs("color"_a = "magenta", "label"_a = "cubic 1st derivative"));
    ax_derivative.plot(
      Args(xs, pchip_first), Kwargs("color"_a = "darkcyan", "label"_a = "pchip 1st derivative"));
    ax_derivative.plot(
      Args(xs, cubic_second),
      Kwargs("color"_a = "mediumpurple", "linestyle"_a = "--", "label"_a = "cubic 2nd derivative"));
    ax_derivative.plot(
      Args(xs, pchip_second),
      Kwargs("color"_a = "seagreen", "linestyle"_a = "--", "label"_a = "pchip 2nd derivative"));
    ax_derivative.grid();
    ax_derivative.legend();
    ax_derivative.set_title(Args("Derivatives"));
    ax_derivative.set_xlim(Args(-0.2, 6.2));

    fig.tight_layout();
    plt.show();
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
}
