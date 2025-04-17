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

#ifndef AUTOWARE__TRAJECTORY__DETAIL__INTERPOLATED_ARRAY_HPP_
#define AUTOWARE__TRAJECTORY__DETAIL__INTERPOLATED_ARRAY_HPP_

#include "autoware/trajectory/detail/logging.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory::detail
{

/**
 * @brief Class representing an array with interpolatable values that can be manipulated.
 * @tparam T The type of values stored in the array.
 */
template <typename T>
class InterpolatedArray
{
  using InterpolatorType = interpolator::InterpolatorInterface<T>;

private:
  std::vector<double> bases_;
  std::vector<T> values_;
  std::shared_ptr<interpolator::InterpolatorInterface<T>> interpolator_;
  std::function<void(const double s)> base_addition_callback_slot_{nullptr};

public:
  /**
   * @brief Construct a InterpolatedArray with a given interpolator.
   * @param interpolator Shared pointer to the interpolator.
   */
  explicit InterpolatedArray(const std::shared_ptr<InterpolatorType> & interpolator)
  : interpolator_(interpolator)
  {
  }

  /**
   * @brief Copy constructor.
   * @param other The InterpolatedArray to copy from.
   */
  InterpolatedArray(const InterpolatedArray & other)
  : bases_(other.bases_),
    values_(other.values_),
    interpolator_(other.interpolator_->clone()),
    base_addition_callback_slot_(other.base_addition_callback_slot_)
  {
  }

  InterpolatedArray(InterpolatedArray && other) = default;

  interpolator::InterpolationResult build(
    const std::vector<double> & bases, const std::vector<T> & values)
  {
    bases_ = bases;
    values_ = values;
    return interpolator_->build(bases_, values_);
  }

  interpolator::InterpolationResult build(
    const std::vector<double> & bases, std::vector<T> && values)
  {
    bases_ = bases;
    values_ = std::move(values);
    return interpolator_->build(bases_, values_);
  }

  /**
   * @brief Move constructor.
   * @param other The InterpolatedArray to move from.
   */
  InterpolatedArray & operator=(InterpolatedArray && other) = default;

  /**
   * @brief Copy assignment operator.
   * @param other The InterpolatedArray to copy from.
   * @return Reference to this InterpolatedArray.
   */
  InterpolatedArray & operator=(const InterpolatedArray & other)
  {
    if (this != &other) {
      bases_ = other.bases_;
      values_ = other.values_;
      interpolator_ = other.interpolator_->clone();
      base_addition_callback_slot_ = other.base_addition_callback_slot_;
    }
    return *this;
  }

  // Destructor
  ~InterpolatedArray() = default;

  /**
   * @brief add the callback function to be executed when a new base is added to this class
   */
  void connect_base_addition_callback(std::function<void(const double s)> && signal)
  {
    base_addition_callback_slot_ = std::move(signal);
  }

  /**
   * @brief Get the start value of the base.
   * @return The start value.
   */
  double start() const { return bases_.front(); }

  /**
   * @brief Get the end value of the base.
   * @return The end value.
   */
  double end() const { return bases_.at(bases_.size() - 1); }

  class Segment
  {
    friend class InterpolatedArray;

    const double start_;
    const double end_;
    InterpolatedArray<T> & parent_;
    Segment(InterpolatedArray<T> & parent, const double start, const double end)
    : start_(start), end_(end), parent_(parent)
    {
    }

  public:
    void set(const T & value)
    {
      std::vector<double> & bases = parent_.bases_;
      std::vector<T> & values = parent_.values_;

      auto insert_new_base_if_not_present = [&](const double new_base) -> size_t {
        auto it = std::lower_bound(bases.begin(), bases.end(), new_base);
        size_t index = std::distance(bases.begin(), it);

        if (it != bases.end() && *it == new_base) {
          // Return the index if the value already exists
          return index;
        }  // Insert into bases
        bases.insert(it, new_base);

        // execute the callback to notify that a new base has been added
        if (parent_.base_addition_callback_slot_) {
          std::invoke(parent_.base_addition_callback_slot_, new_base);
        }

        // Insert into values at the corresponding position
        values.insert(values.begin() + index, value);
        return index;
      };

      // Insert the start value if not present
      size_t start_index = insert_new_base_if_not_present(start_);

      // Insert the end value if not present
      size_t end_index = insert_new_base_if_not_present(end_);

      // Ensure the indices are in ascending order
      if (start_index > end_index) {
        std::swap(start_index, end_index);
      }

      // Set the values in the specified range
      std::fill(values.begin() + start_index, values.begin() + end_index + 1, value);

      const auto success = parent_.interpolator_->build(bases, values);
      if (!success) {
        throw std::runtime_error(
          "Failed to build interpolator.");  // This Exception should not be thrown.
      }
    }
  };

  // TODO(soblin): how to insert "linear" or "decelerating" velocity profile ?
  /**
   * @brief Get a Segment object to set values in a specific range.
   * @param start Start of the range.
   * @param end End of the range.
   * @return RangeSetter object.
   */
  Segment range(double start, double end)
  {
    if (start < this->start() || end > this->end()) {
      RCLCPP_WARN(
        get_logger(), "The range [%f, %f] is out of the array range [%f, %f]", start, end,
        this->start(), this->end());
      start = std::max(start, this->start());
      end = std::min(end, this->end());
    }
    return Segment{*this, start, end};
  }

  /**
   * @brief Assign a value to the entire array.
   * @param value Value to be assigned.
   * @return Reference to the InterpolatedArray object.
   */
  InterpolatedArray & operator=(const T & value)
  {
    std::fill(values_.begin(), values_.end(), value);
    const auto success = interpolator_->build(bases_, values_);
    if (!success) {
      throw std::runtime_error(
        "Failed to build interpolator.");  // This Exception should not be thrown.
    }
    return *this;
  }

  /**
   * @brief Compute the interpolated value at a given position.
   * @param x The position to compute the value at.
   * @return The interpolated value.
   */
  T compute(const double x) const { return interpolator_->compute(x); }

  /**
   * @brief Get the underlying data of the array.
   * @return A pair containing the axis and values.
   */
  std::pair<std::vector<double>, std::vector<T>> get_data() const { return {bases_, values_}; }
};

}  // namespace autoware::experimental::trajectory::detail

#endif  // AUTOWARE__TRAJECTORY__DETAIL__INTERPOLATED_ARRAY_HPP_
