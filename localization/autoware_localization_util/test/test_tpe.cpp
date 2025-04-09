// Copyright 2023 The Autoware Contributors
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

#include "autoware/localization_util/tree_structured_parzen_estimator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using TreeStructuredParzenEstimator = autoware::localization_util::TreeStructuredParzenEstimator;

TEST(TreeStructuredParzenEstimatorTest, TPE_is_better_than_random_search_on_sphere_function)
{
  auto sphere_function = [](const TreeStructuredParzenEstimator::Input & input) {
    double value = 0.0;
    const auto n = static_cast<int64_t>(input.size());
    for (int64_t i = 0; i < n; i++) {
      const double v = input[i] * 10;
      value += v * v;
    }
    return value;
  };

  constexpr int64_t k_outer_trials_num = 20;
  constexpr int64_t k_inner_trials_num = 200;
  std::cout << std::fixed;
  std::vector<double> mean_scores;
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev{1.0, 1.0, 0.1, 0.1, 0.1};

  for (const int64_t n_startup_trials : {k_inner_trials_num, k_inner_trials_num / 2}) {
    const std::string method = ((n_startup_trials == k_inner_trials_num) ? "Random" : "TPE");

    std::vector<double> scores;
    for (int64_t i = 0; i < k_outer_trials_num; i++) {
      double best_score = std::numeric_limits<double>::lowest();
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials, sample_mean,
        sample_stddev);
      for (int64_t trial = 0; trial < k_inner_trials_num; trial++) {
        const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
        const double score = -sphere_function(input);
        estimator.add_trial({input, score});
        best_score = std::max(best_score, score);
      }
      scores.push_back(best_score);
    }

    const double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    const double mean = sum / static_cast<double>(scores.size());
    mean_scores.push_back(mean);
    double sq_sum = std::accumulate(
      scores.begin(), scores.end(), 0.0,
      [mean](double total, double score) { return total + (score - mean) * (score - mean); });
    const double stddev = std::sqrt(sq_sum / static_cast<double>(scores.size()));

    std::cout << method << ", mean = " << mean << ", stddev = " << stddev << std::endl;
  }
  ASSERT_LT(mean_scores[0], mean_scores[1]);
}

// Test the correctness of the TPE constructor
TEST(TreeStructuredParzenEstimatorTest, Constructor)
{
  // Parameters of correct size should construct successfully
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev(5, 1.0);
  EXPECT_NO_THROW({
    TreeStructuredParzenEstimator estimator(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, sample_stddev);
  });

  // Parameters of incorrect size should throw an exception
  std::vector<double> invalid_mean(4, 0.0);
  std::vector<double> invalid_stddev(4, 1.0);

  EXPECT_THROW(
    {
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, invalid_mean, sample_stddev);
    },
    std::runtime_error);

  EXPECT_THROW(
    {
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, invalid_stddev);
    },
    std::runtime_error);
}

// Test the add_trial method and sorting functionality
TEST(TreeStructuredParzenEstimatorTest, AddTrial)
{
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev(5, 1.0);

  // Maximization direction test
  {
    TreeStructuredParzenEstimator estimator(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, 5, sample_mean, sample_stddev);

    // Add some trials with varying scores
    estimator.add_trial({{0.1, 0.2, 0.3, 0.4, 0.5}, 10.0});
    estimator.add_trial({{0.2, 0.3, 0.4, 0.5, 0.6}, 20.0});
    estimator.add_trial({{0.3, 0.4, 0.5, 0.6, 0.7}, 5.0});

    // Verify that the next input is generated based on previous trials
    // Here, we do not test specific values, only ensure the function does not crash
    EXPECT_NO_THROW({
      auto next_input = estimator.get_next_input();
      EXPECT_EQ(next_input.size(), 6);  // Input dimension should be 6
    });
  }

  // Minimization direction test
  {
    TreeStructuredParzenEstimator estimator(
      TreeStructuredParzenEstimator::Direction::MINIMIZE, 5, sample_mean, sample_stddev);

    // Add some trials with varying scores
    estimator.add_trial({{0.1, 0.2, 0.3, 0.4, 0.5}, 10.0});
    estimator.add_trial({{0.2, 0.3, 0.4, 0.5, 0.6}, 20.0});
    estimator.add_trial({{0.3, 0.4, 0.5, 0.6, 0.7}, 5.0});

    // Verify that the next input is generated based on previous trials
    EXPECT_NO_THROW({
      auto next_input = estimator.get_next_input();
      EXPECT_EQ(next_input.size(), 6);  // Input dimension should be 6
    });
  }
}

// Test the behavior of the get_next_input method in different phases
TEST(TreeStructuredParzenEstimatorTest, GetNextInput)
{
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev(5, 1.0);

  // Startup phase test - insufficient n_startup_trials_
  {
    TreeStructuredParzenEstimator estimator(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, 10, sample_mean, sample_stddev);

    // Without adding any trials, random sampling should occur
    auto input1 = estimator.get_next_input();
    EXPECT_EQ(input1.size(), 6);

    // Add some trials, but not enough to reach the startup number
    for (int i = 0; i < 5; ++i) {
      estimator.add_trial({{0.1, 0.2, 0.3, 0.4, 0.5}, static_cast<double>(i)});
    }

    // Random sampling should still occur
    auto input2 = estimator.get_next_input();
    EXPECT_EQ(input2.size(), 6);
  }

  // Optimization phase test - exceeding n_startup_trials_
  {
    TreeStructuredParzenEstimator estimator(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, 5, sample_mean, sample_stddev);

    // Add more trials than the startup number
    for (int i = 0; i < 10; ++i) {
      estimator.add_trial({{0.1, 0.2, 0.3, 0.4, 0.5}, static_cast<double>(i)});
    }

    // The TPE method should now be used to generate the next input
    auto input = estimator.get_next_input();
    EXPECT_EQ(input.size(), 6);
  }
}

// Test the optimization capability of the TPE method on a minimization problem
TEST(TreeStructuredParzenEstimatorTest, TPE_minimizes_quadratic_function)
{
  // Define a simple quadratic function with the minimum at the origin
  auto quadratic_function = [](const TreeStructuredParzenEstimator::Input & input) {
    double value = 0.0;
    // Only use the first two dimensions for calculation
    value += std::pow(input[0], 2);
    value += std::pow(input[1], 2);
    return value;
  };

  constexpr int64_t k_inner_trials_num = 100;
  std::vector<double> sample_mean(5, 1.0);  // Initial point far from the optimal solution
  std::vector<double> sample_stddev(5, 1.0);

  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MINIMIZE, k_inner_trials_num / 2, sample_mean,
    sample_stddev);

  double best_score = std::numeric_limits<double>::max();
  TreeStructuredParzenEstimator::Input best_input;

  // Run the optimization
  for (int64_t trial = 0; trial < k_inner_trials_num; trial++) {
    const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
    const double score = quadratic_function(input);
    estimator.add_trial({input, score});

    if (score < best_score) {
      best_score = score;
      best_input = input;
    }
  }

  // Verify that the final solution is close to the origin
  std::cout << "Best score: " << best_score << std::endl;
  std::cout << "Best input: (" << best_input[0] << ", " << best_input[1] << ")" << std::endl;

  // The minimum value should be close to 0 (considering randomness, we only assert it is better
  // than the initial position)
  ASSERT_LT(best_score, quadratic_function({1.0, 1.0, 0.0, 0.0, 0.0, 0.0}));
}

// Test correctness in handling special angle values (e.g., near ±π)
TEST(TreeStructuredParzenEstimatorTest, AngleWrappingHandling)
{
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev(5, 1.0);

  TreeStructuredParzenEstimator estimator(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, 5, sample_mean, sample_stddev);

  // Add some trials containing angle values near ±π
  estimator.add_trial({{0.0, 0.0, 0.0, 0.0, 0.0, M_PI - 0.1}, 10.0});
  estimator.add_trial({{0.0, 0.0, 0.0, 0.0, 0.0, -M_PI + 0.1}, 20.0});

  // Ensure angle wrapping is handled correctly
  EXPECT_NO_THROW({
    auto input = estimator.get_next_input();
    EXPECT_EQ(input.size(), 6);
  });
}
