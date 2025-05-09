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

#pragma once

#include <chrono>
#include <optional>
#include <vector>

template <typename ItemType>
class GenericBuffer
{
public:
  using TimePoint = std::chrono::system_clock::time_point;
  using Buffer = std::vector<ItemType>;

  virtual ~GenericBuffer() = default;

  virtual std::optional<ItemType> getNearestActiveItem() const = 0;

protected:
  Buffer buffer_;
  double min_off_duration_;
  double min_on_duration_;

  GenericBuffer() = default;

  GenericBuffer(double min_off_duration, double min_on_duration)
  : buffer_(), min_off_duration_(min_off_duration), min_on_duration_(min_on_duration)
  {
  }
}
