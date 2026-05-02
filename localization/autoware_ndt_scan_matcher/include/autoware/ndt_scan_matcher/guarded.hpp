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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__GUARDED_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__GUARDED_HPP_

#include <mutex>
#include <utility>

namespace autoware::ndt_scan_matcher
{

// A value that can only be accessed under a mutex lock, via a callback.
//
// Unlike a bare std::mutex + member variable, the protected value is private,
// so the compiler enforces that every access goes through with().
// The callback-based API ensures the reference cannot escape the lock scope:
// the return type uses `auto` deduction, which strips references by value.
//
// This is the same idea as Rust's Mutex<T> and folly::Synchronized<T>,
// adapted for C++ without a borrow checker.
//
// Usage:
//   Guarded<std::optional<Point>> position_;
//
//   // write
//   position_.with([&](auto & pos) { pos = new_value; });
//
//   // read (returned by value — cannot leak a reference)
//   auto copy = position_.with([](const auto & pos) { return pos; });
//
//   // extended access
//   ndt_.with([&](auto & ndt_ptr) {
//     ndt_ptr->setInputSource(scan);
//     ndt_ptr->align(output, initial_pose);
//   });  // lock released
//
template <typename T>
class Guarded
{
public:
  Guarded() = default;

  template <typename... Args>
  explicit Guarded(Args &&... args) : value_(std::forward<Args>(args)...)
  {
  }

  Guarded(const Guarded &) = delete;
  Guarded & operator=(const Guarded &) = delete;
  Guarded(Guarded &&) = delete;
  Guarded & operator=(Guarded &&) = delete;

  ~Guarded() = default;

  // The only way to access the value. The reference to value_ exists only inside f.
  // Return type uses `auto` deduction, which strips top-level references,
  // preventing accidental reference leaking through the return value.
  template <typename F>
  auto with(F && f)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return f(value_);
  }

private:
  std::mutex mtx_;
  T value_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__GUARDED_HPP_
