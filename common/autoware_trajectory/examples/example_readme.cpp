// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/interpolator/stairstep.hpp"

#include <autoware/pyplot/pyplot.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <random>
#include <vector>

// NOLINTBEGIN
int main_cubic_normal()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, ax] = plt.subplots();

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()

  // The result may be `InterpolationFailure` type if the input is not suitable
  // for this interpolator
  if (!result) {
    return 0;
  }

  // If successful, the interpolator is available.
  // In this case the cubic spline coefficients are calculated.
  const CubicSpline & trajectory = result.value();

  // arbitrary value can be computed continuously
  std::vector<double> x;
  std::vector<double> y;
  for (double s = xs.front(); s <= xs.back(); s += 0.05) {
    x.push_back(s);
    y.push_back(trajectory.compute(s));
  }
  ax.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "interpolated(cubic spline)"));

  // interpolate multiple values
  const auto ys2 = trajectory.compute(xs);
  ax.scatter(Args(xs, ys2), Kwargs("color"_a = "red", "label"_a = "underlying points"));

  ax.scatter(
    Args(1.5, trajectory.compute(1.5)),
    Kwargs("color"_a = "green", "marker"_a = "o", "label"_a = "x=1.5"));
  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();
  fig.tight_layout();
  plt.savefig(Args("cubic_spline.svg"));

  return 0;
}

int main_cubic_error()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  // Give only 3 points for CubicSpline, which is infeasible
  std::vector<double> xs = {0.0, 1.0, 2.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()

  // Access to the error reason
  if (!result) {
    // says "base size 3 is less than minimum required 4"
    std::cout << result.error().what << std::endl;
  }
  return 0;
}

int main_akima()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::AkimaSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<AkimaSpline, InterpolationFailure> result =
    AkimaSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "akima spline"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("akima_spline.svg"));
  return 0;
}

int main_cubic()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::CubicSpline;
  using autoware::trajectory::interpolator::InterpolationFailure;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<CubicSpline, InterpolationFailure> result =
    CubicSpline::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "cubic spline"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("cubic_spline.svg"));
  return 0;
}

int main_linear()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Linear;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<Linear, InterpolationFailure> result = Linear::Builder()
                                                              .set_bases(xs)   // set the base
                                                              .set_values(ys)  // set the value
                                                              .build();  // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "linear"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("linear.svg"));
  return 0;
}

int main_stairstep()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  auto noise = [&]() { return dis(gen); };
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto & ax1 = axes[0];
  auto & ax2 = axes[1];

  using autoware::trajectory::interpolator::InterpolationFailure;
  using autoware::trajectory::interpolator::Stairstep;

  std::vector<double> xs = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ys = {0.0 + noise(), 1.0 + noise(), 2.0 + noise(),
                            3.0 + noise(), 4.0 + noise(), 5.0 + noise()};

  // You need to use the `Builder` to try to construct the interpolator
  const tl::expected<Stairstep<double>, InterpolationFailure> result =
    Stairstep<double>::Builder()
      .set_bases(xs)   // set the base
      .set_values(ys)  // set the value
      .build();        // finally, call build()
  const auto & trajectory = result.value();
  const auto x = trajectory.base_arange(0.05);
  const auto y = trajectory.compute(x);
  const auto dy = trajectory.compute_first_derivative(x);
  const auto ddy = trajectory.compute_second_derivative(x);
  ax1.plot(Args(x, y), Kwargs("color"_a = "purple", "label"_a = "stairstep"));
  ax1.scatter(Args(xs, ys), Kwargs("color"_a = "red", "marker"_a = "o", "label"_a = "underlying"));
  ax1.legend();
  ax1.grid();
  ax1.set_aspect(Args("equal"));
  ax1.set_xlim(Args(-1.0, 6.0));
  ax1.set_ylim(Args(-1.0, 6.0));

  ax2.plot(Args(x, dy), Kwargs("color"_a = "magenta", "label"_a = "1st derivative"));
  ax2.plot(Args(x, ddy), Kwargs("color"_a = "purple", "label"_a = "2nd derivative"));
  ax2.legend();
  ax2.grid();
  ax2.set_aspect(Args("equal"));
  ax2.set_xlim(Args(-1.0, 6.0));
  ax2.set_ylim(Args(-3.0, 4.0));

  fig.tight_layout();
  plt.savefig(Args("stairstep.svg"));
  return 0;
}

int main()
{
  pybind11::scoped_interpreter guard{};

  main_cubic_normal();
  main_cubic_error();
  main_akima();
  main_cubic();
  main_linear();
  main_stairstep();
}
// NOLINTEND
