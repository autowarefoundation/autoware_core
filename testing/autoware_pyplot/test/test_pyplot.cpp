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

// cspell:ignore yerr

#include <autoware/pyplot/patches.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware/pyplot/text.hpp>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
/*
  very weirdly, you must include <pybind11/stl.h> to pass vector. Although the code
  compiles without it, the executable crashes at runtime
 */
#include <pybind11/stl.h>

#include <cstddef>
#include <vector>

TEST(PyPlot, single_plot)
{
  // NOTE: somehow, running multiple tests simultaneously causes the python interpreter to crash
  py::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();
  {
    plt.plot(Args(std::vector<int>({1, 3, 2, 4})), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
    plt.xlabel(Args("x-title"));
    plt.ylabel(Args("y-title"));
    plt.title(Args("title"));
    plt.xlim(Args(0, 5));
    plt.ylim(Args(0, 5));
    plt.grid(Args(true));
    plt.savefig(Args("test_single_plot.png"));
  }
  {
    auto [fig, axes] = plt.subplots(1, 2);
    EXPECT_EQ(axes.size(), 2);
    auto & ax1 = axes[0];
    auto & ax2 = axes[1];

    auto c =
      autoware::pyplot::Circle(Args(py::make_tuple(0, 0), 0.5), Kwargs("fc"_a = "g", "ec"_a = "r"));
    ax1.add_patch(Args(c.unwrap()));

    auto e = autoware::pyplot::Ellipse(
      Args(py::make_tuple(-0.25, 0), 0.5, 0.25), Kwargs("fc"_a = "b", "ec"_a = "y"));
    ax1.add_patch(Args(e.unwrap()));

    auto r = autoware::pyplot::Rectangle(
      Args(py::make_tuple(0, 0), 0.25, 0.5), Kwargs("ec"_a = "#000000", "fill"_a = false));
    ax2.add_patch(Args(r.unwrap()));

    ax1.set_aspect(Args("equal"));
    ax2.set_aspect(Args("equal"));
    plt.savefig(Args("test_double_plot.svg"));
  }
  {
    // subplots(r, c) NxM (r > 1 && c > 1) row-major flattening branch
    const int rows = 2;
    const int cols = 3;
    auto [fig, axes] = plt.subplots(rows, cols);
    EXPECT_EQ(axes.size(), static_cast<std::size_t>(rows * cols));
    // every flattened cell must be a usable Axes wrapper
    for (auto & ax : axes) {
      ax.set_xlim(Args(0, 1));
      const auto [xmin, xmax] = ax.get_xlim();
      EXPECT_DOUBLE_EQ(xmin, 0.0);
      EXPECT_DOUBLE_EQ(xmax, 1.0);
    }
  }
  {
    // subplots(1, 3) Nx1 / 1xN flattening branch
    auto [fig, axes] = plt.subplots(1, 3);
    EXPECT_EQ(axes.size(), 3);
  }
  {
    // subplots(1, 1) single-Axes branch
    auto [fig, axes] = plt.subplots(1, 1);
    EXPECT_EQ(axes.size(), 1);
  }
  {
    // newly wired Axes forwarders: errorbar / bar_label / contourf
    auto [fig, axes] = plt.subplots(1, 1);
    auto & ax = axes[0];

    // errorbar returns an ErrorbarContainer
    ax.errorbar(
      Args(std::vector<double>({0.0, 1.0, 2.0}), std::vector<double>({0.0, 1.0, 4.0})),
      Kwargs("yerr"_a = 0.5));

    // bar() + bar_label() must round-trip without a link error
    auto bars =
      ax.unwrap().attr("bar")(std::vector<double>({0.0, 1.0}), std::vector<double>({1.0, 2.0}));
    ax.bar_label(Args(bars));

    // contourf uses the eagerly loaded contourf_attr
    ax.contourf(Args(
      std::vector<std::vector<double>>({{0.0, 1.0}, {1.0, 2.0}}),
      std::vector<std::vector<double>>({{0.0, 1.0}, {1.0, 2.0}}),
      std::vector<std::vector<double>>({{1.0, 2.0}, {3.0, 4.0}})));
    plt.savefig(Args("test_axes_forwarders.png"));
  }
  {
    // Text::set_rotation: load_attrs() is now wired into the constructor, so the
    // forwarder resolves and applies the rotation to the underlying Text artist.
    auto [fig, axes] = plt.subplots(1, 1);
    auto & ax = axes[0];
    auto text_obj = ax.text(Args(0.5, 0.5, "label"));
    autoware::pyplot::Text txt(text_obj.unwrap());
    txt.set_rotation(Args(45.0));
    EXPECT_DOUBLE_EQ(txt.unwrap().attr("get_rotation")().cast<double>(), 45.0);
  }
}
