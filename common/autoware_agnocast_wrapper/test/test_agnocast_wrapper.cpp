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

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

namespace autoware::agnocast_wrapper
{
namespace
{

// ---------------------------------------------------------------------------
// detail::parse_enable_agnocast: ENABLE_AGNOCAST environment-value parsing.
// ---------------------------------------------------------------------------

TEST(ParseEnableAgnocast, NullptrParsesToZero)
{
  EXPECT_EQ(detail::parse_enable_agnocast(nullptr), 0);
}

TEST(ParseEnableAgnocast, EmptyStringParsesToZero)
{
  EXPECT_EQ(detail::parse_enable_agnocast(""), 0);
}

TEST(ParseEnableAgnocast, ZeroStringParsesToZero)
{
  EXPECT_EQ(detail::parse_enable_agnocast("0"), 0);
}

TEST(ParseEnableAgnocast, OneStringParsesToOne)
{
  EXPECT_EQ(detail::parse_enable_agnocast("1"), 1);
}

TEST(ParseEnableAgnocast, LeadingWhitespaceIsSkipped)
{
  // std::atoi skips leading whitespace before parsing the integer.
  EXPECT_EQ(detail::parse_enable_agnocast("  1"), 1);
  EXPECT_EQ(detail::parse_enable_agnocast("\t0"), 0);
}

TEST(ParseEnableAgnocast, GarbageLeadingTextParsesToZero)
{
  // std::atoi returns 0 when no leading digits are present.
  EXPECT_EQ(detail::parse_enable_agnocast("yes"), 0);
  EXPECT_EQ(detail::parse_enable_agnocast("true"), 0);
  EXPECT_EQ(detail::parse_enable_agnocast("on"), 0);
}

TEST(ParseEnableAgnocast, TrailingGarbageIsIgnored)
{
  // std::atoi parses the leading integer and ignores the remainder.
  EXPECT_EQ(detail::parse_enable_agnocast("1abc"), 1);
  EXPECT_EQ(detail::parse_enable_agnocast("0xyz"), 0);
}

TEST(ParseEnableAgnocast, NegativeValueParsesToNegative)
{
  // Only the literal value 1 enables Agnocast; a negative value is not 1.
  EXPECT_EQ(detail::parse_enable_agnocast("-1"), -1);
}

TEST(ParseEnableAgnocast, OtherPositiveValuesAreNotOne)
{
  // use_agnocast() compares the parsed value against exactly 1, so 2 stays 2.
  EXPECT_EQ(detail::parse_enable_agnocast("2"), 2);
}

// ---------------------------------------------------------------------------
// detail::validate_timer_period: timer-period precondition boundary.
// ---------------------------------------------------------------------------

TEST(ValidateTimerPeriod, ZeroIsAccepted)
{
  EXPECT_NO_THROW(detail::validate_timer_period(std::chrono::nanoseconds::zero()));
}

TEST(ValidateTimerPeriod, PositivePeriodIsAccepted)
{
  EXPECT_NO_THROW(detail::validate_timer_period(std::chrono::milliseconds(10)));
}

TEST(ValidateTimerPeriod, LargeButNotMaxIsAccepted)
{
  const auto just_below_max =
    std::chrono::nanoseconds{std::numeric_limits<std::chrono::nanoseconds::rep>::max() - 1};
  EXPECT_NO_THROW(detail::validate_timer_period(just_below_max));
}

TEST(ValidateTimerPeriod, NegativePeriodThrowsInvalidArgument)
{
  EXPECT_THROW(detail::validate_timer_period(std::chrono::nanoseconds{-1}), std::invalid_argument);
}

TEST(ValidateTimerPeriod, MaxPeriodThrowsInvalidArgument)
{
  EXPECT_THROW(
    detail::validate_timer_period(std::chrono::nanoseconds::max()), std::invalid_argument);
}

}  // namespace
}  // namespace autoware::agnocast_wrapper
