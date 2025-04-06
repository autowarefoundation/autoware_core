# Autoware Trajectory

This package provides classes to manage/manipulate Trajectory.

## Overview

### Interpolators

The interpolator class interpolates given `bases` and `values`. Following interpolators are implemented.

- Linear
- AkimaSpline
- CubicSpline
- NearestNeighbor
- Stairstep

![interpolators](./images/overview/interpolators.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/overview/interpolators.drawio.svg") }})

The builder internally executes interpolation and return the result in the form of `expected<T, E>`. If successful, it contains the interpolator object.

```cpp title="./examples/example_readme.cpp:48:67"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:48:67
--8<--
```

Otherwise it contains the error object representing the failure reason. In the below snippet, cubic spline interpolation fails because the number of input points is 3, which is below the `minimum_required_points() = 4` of `CubicSpline`.

```cpp title="./examples/example_readme.cpp:109:119"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:109:119
--8<--
```

In such cases the result `expected` object contains `InterpolationFailure` type with an error message like **"base size 3 is less than minimum required 4"**.

### Trajectory class

The _Trajectory_ class provides mathematical continuous representation and object oriented interface for discrete array of following point types

- [x] `geometry_msgs::Point`
- [x] `geometry_msgs::Pose`
- [x] `autoware_planning_msgs::PathPoint`
- [x] `autoware_planning_msgs::PathPointWithLaneId`
- [x] `autoware_planning_msgs::TrajectoryPoint`
- [ ] `lanelet::ConstPoint3d`

by interpolating the given _underlying_ points. Once built, arbitrary point on the curve is continuously parametrized by a single `s` coordinate.

```cpp title="./examples/example_readme.cpp:547:562"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:547:562
--8<--
```

![overview_trajectory](./images/overview/trajectory.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/overview/trajectory.drawio.svg") }})

## Nomenclature

This section introduces strict definition of several words used in this package to clarify the description of API and help the developers understand and grasp the geometric meaning of algorithms.

| Word            | Meaning                                                                                                                                                                                                                                                                                                                                                                          | Illustration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| --------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `curve`         | `curve` is an oriented bounded curve denoted as `(x(s), y(s), z(s))` with additional properties, parameterized by `s` (`s = 0` at the start).                                                                                                                                                                                                                                    | ![curve](./images/nomenclature/curve.drawio.svg)<br>[View in Drawio]({{ drawio("/common/autoware_trajectory/images/nomenclature/curve.drawio.svg") }})<br>There are 5 `underlying` points<br>$\mathrm{P0} = (0, 0, 0)$<br>$\mathrm{P1} = (1/ \sqrt{2}, 1/ \sqrt{2}, 0)$<br>$\mathrm{P2} = (1/ \sqrt{2}, 1+1/ \sqrt{2}, 0)$<br>$\mathrm{P3} = (2/ \sqrt{2}, 1+2/ \sqrt{2}, 0)$<br>$\mathrm{P4} = (2/ \sqrt{2} + 1/ \sqrt{6}, 1+2/ \sqrt{2} + 1 / \sqrt{3}, 1 / \sqrt{2})$<br>and the `arc length` between each interval is $1, 2, 1, 1$ respectively, so $\mathrm{start} = 0$ and $\mathrm{end} = 5$. |
| `underlying`    | `underlying` points of a curve refers to the list of 3D points from which the curve was interpolated.                                                                                                                                                                                                                                                                            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `arc length`[m] | `arc length` denotes the approximate **3D** length of of a curve and is computed based on the discrete `underlying` points.                                                                                                                                                                                                                                                      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `s`[m]          | `s` denotes the **3D** `arc length` coordinate starting from the base point (mostly the start point) of the curve and a point is identified by `trajectory(s)`.<br>Due to this definition, the actual _curve length_ and `arc length` have subtle difference as illustrated.                                                                                                     | ![approximation](./images/nomenclature/approximation.drawio.svg)<br>[View in Drawio]({{ drawio("/common/autoware_trajectory/images/nomenclature/approximation.drawio.svg") }})<br>The point for $s = 0.5$ is the purple dot, but the _curve length_ from $\mathrm{P0}$ to this point does not equal to $0.5$.<br>The exact _curve length_ is $\int \sqrt{(\frac{dx}{dt})^2 + (\frac{dy}{dt})^2 + (\frac{dz}{dt})^2} dt$, which cannot be obtained in an analytical closed form.                                                                                                                      |
| `curvature`     | `curvature` is computed **using only X-Y 2D coordinate**. This is based on the normal and natural assumption that _roads are flat_. Mathematically, it asserts that [Gaussian curvature](https://en.wikipedia.org/wiki/Gaussian_curvature) of road is uniformly 0.<br>The sign of curvature is positive if the center of turning circle is on the left side, otherwise negative. | ![curvature](./images/nomenclature/curvature.drawio.svg)<br>[View in Drawio]({{ drawio("/common/autoware_trajectory/images/nomenclature/curvature.drawio.svg") }})                                                                                                                                                                                                                                                                                                                                                                                                                                   |

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

```cpp title="./examples/example_readme.cpp:137:151"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:137:151
--8<--
```

![akima_spline](./images/akima_spline.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/akima_spline.drawio.svg") }})

`CubicSpline` requires at least **4** points to interpolate.

```cpp title="./examples/example_readme.cpp:192:201"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:192:201
--8<--
```

![cubic_spline](./images/cubic_spline.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/cubic_spline.drawio.svg") }})

`Linear` requires at least **2** points to interpolate.

```cpp title="./examples/example_readme.cpp:242:250"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:242:250
--8<--
```

![linear](./images/linear.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/linear.drawio.svg") }})

`StairStep` requires at least **2** points to interpolate.

```cpp title="./examples/example_readme.cpp:291:300"
--8<--
common/autoware_trajectory/examples/example_readme.cpp:291:300
--8<--
```

![stairstep](./images/stairstep.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/stairstep.drawio.svg") }})

### Trajectory class

Several `Trajectory<T>` are defined in the following inheritance hierarchy according to the sub object relationships.

![hierarchy](./images/nomenclature/trajectory_hierarchy.drawio.svg)
[View in Drawio]({{ drawio("/common/autoware_trajectory/images/nomenclature/trajectory_hierarchy.drawio.svg") }})

Each derived class in the diagram inherits the methods of all of its descending subclasses. For example, all of the classes have the methods like `length()`, `curvature()` in common.

| Header/Class                                                                                            | method                                    | description                                                                                                                                        | illustration                                                                                                                                                                                                                             |
| ------------------------------------------------------------------------------------------------------- | ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `<autoware/trajectory/point.hpp>`<br><ul><li>`Trajectory<geometry_msgs::msg::Point>::Builder`</li></ul> | `Builder()`                               | set default interpolator setting as follows.<br><ul><li>`x, y`: Cubic</li><li>`z`: Linear</li></ul>                                                |                                                                                                                                                                                                                                          |
|                                                                                                         | `set_xy_interpolator<InterpolatorType>()` | set custom interpolator for `x, y`.                                                                                                                |                                                                                                                                                                                                                                          |
|                                                                                                         | `set_z_interpolator<InterpolatorType>()`  | set custom interpolator for `z`.                                                                                                                   |                                                                                                                                                                                                                                          |
|                                                                                                         | `build(const vector<Point> &)`            | return `expected<Trajectory<Point>, InterpolationFailure>` object.                                                                                 |                                                                                                                                                                                                                                          |
| <ul><li>`Trajectory<Point>`</li></ul>                                                                   | `base_arange(const double step)`          | return vector of `s` values starting from `start`, with the interval of `step`, including `end`. Thus the return value has at least the size of 2. |                                                                                                                                                                                                                                          |
|                                                                                                         | `length()`                                | return the total `arc length` of the trajectory.                                                                                                   | ![curve](./images/nomenclature/curve.drawio.svg)<br>[View in Drawio]({{ drawio("/common/autoware_trajectory/images/nomenclature/curve.drawio.svg") }})<br>`length()` is $5.0$ because it computes the sum of the length of dotted lines. |
|                                                                                                         | `azimuth(const double s)`                 | return the tangent angle at given `s` coordinate using `std::atan2`.                                                                               | ![azimuth_angle](./images/overview/trajectory.drawio.svg)<br>[View in Drawio]({{ drawio("/common/autoware_trajectory/images/overview/trajectory.drawio.svg") }})                                                                         |
|                                                                                                         | `curvature(const double s)`               | return the `curvature` at given `s` coordinate.                                                                                                    | See above                                                                                                                                                                                                                                |

## Example Usage

This section describes Example Usage of `Trajectory<autoware_planning_msgs::msg::PathPoint>`

- You can also specify interpolation method to `Builder{}` before calling `.build(points)`

  ```cpp
  using autoware::trajectory::interpolator::CubicSpline;

  std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> trajectory =
    Trajectory<autoware_planning_msgs::msg::PathPoint>::Builder{}
      .set_xy_interpolator<CubicSpline>()  // Set interpolator for x-y plane
      .build(points);
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
