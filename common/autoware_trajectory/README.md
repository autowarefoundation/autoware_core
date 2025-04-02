# Autoware Trajectory

This package provides classes to manage/manipulate Trajectory.

## Nomenclature

TODO

## Overview

### Interpolators

![interpolators](./images/interpolators.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/interpolators.drawio.svg") }})

Given `bases` and `values`, the builder internally executes interpolation and return the result in the form of `expected<T, E>`. If successful, it contains the interpolator object.

```cpp title="./examples/example_readme.cpp:43:62"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:43:62
--8<--
```

Otherwise it contains the error object representing the failure reason. In the below snippet, cubic spline interpolation fails because the number of input points is 3, which is below the `minimum_required_points() = 4` of `CubicSpline`.

```cpp title="./examples/example_readme.cpp:104:114"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:104:114
--8<--
```

In such cases the result `expected` object contains `InterpolationFailure` type with an error message like **"base size 3 is less than minimum required 4"**.

## API

### Interpolators

| Class            | method/function                                 | description                                                                                    |
| ---------------- | ----------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| Common Functions | `minimum_required_points()`                     | return the number of points required for each concrete interpolator                            |
|                  | `compute(double s) -> T`                        | compute the interpolated value at given base $s$. $s$ is clamped to the underlying base range. |
|                  | `compute(vector<double> s) -> vector<T>`        | compute the interpolated values at for each base values in $s$.                                |
|                  | `compute_first_derivative(double s) -> double`  | compute the first derivative of at given base $s$. $s$ is clamped.                             |
|                  | `compute_second_derivative(double s) -> double` | compute the second derivative of at given base $s$. $s$ is clamped.                            |

`AkimaSpline` requires at least **5** points to interpolate.

```cpp title="./examples/example_readme.cpp:132:146"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:132:146
--8<--
```

![akima_spline](./images/akima_spline.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/akima_spline.drawio.svg") }})

`CubicSpline` requires at least **4** points to interpolate.

```cpp title="./examples/example_readme.cpp:187:196"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:187:196
--8<--
```

![cubic_spline](./images/cubic_spline.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/cubic_spline.drawio.svg") }})

`Linear` requires at least **2** points to interpolate.

```cpp title="./examples/example_readme.cpp:237:245"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:237:245
--8<--
```

![linear](./images/linear.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/linear.drawio.svg") }})

`StairStep` requires at least **2** points to interpolate.

```cpp title="./examples/example_readme.cpp:286:295"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:286:295
--8<--
```

![stairstep](./images/stairstep.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/stairstep.drawio.svg") }})

## Example Usage

This section describes Example Usage of `Trajectory<autoware_planning_msgs::msg::PathPoint>`

- Load Trajectory from point array

  ```cpp
  #include "autoware/trajectory/path_point.hpp"

  ...

  std::vector<autoware_planning_msgs::msg::PathPoint> points = ... // Load points from somewhere

  using autoware::trajectory::Trajectory;

  std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> trajectory =
    Trajectory<autoware_planning_msgs::msg::PathPoint>::Builder{}
      .build(points);
  ```

- You can also specify interpolation method

  ```cpp
  using autoware::trajectory::interpolator::CubicSpline;

  std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> trajectory =
    Trajectory<autoware_planning_msgs::msg::PathPoint>::Builder{}
      .set_xy_interpolator<CubicSpline>()  // Set interpolator for x-y plane
      .build(points);
  ```

- Access point on Trajectory

  ```cpp
  autoware_planning_msgs::msg::PathPoint point = trajectory->compute(1.0);  // Get point at s=0.0. s is distance from start point on Trajectory.
  ```

- Get length of Trajectory

  ```cpp
  double length = trajectory->length();
  ```

- Set 3.0[m] ~ 5.0[m] part of velocity to 0.0

  ```cpp
  trajectory->longitudinal_velocity_mps(3.0, 5.0) = 0.0;
  ```

- Crop Trajectory from 1.0[m] to 2.0[m]

  ```cpp
  trajectory->crop(1.0, 2.0);
  ```

- Restore points

  ```cpp
  std::vector<autoware_planning_msgs::msg::PathPoint> points = trajectory->restore();
  ```
