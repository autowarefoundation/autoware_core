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

#include "test_case.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <string>
namespace fs = std::filesystem;

namespace autoware::experimental
{
class TestNNSearch001 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "autoware_lanelet2_utils")) /
                                "test_data" / "test_nn_search_001.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);

    all_lanelets_ = lanelet_map->laneletLayer | ranges::to<std::vector>();
    rtree_.emplace(lanelet2_utils::LaneletRTree(all_lanelets_));

    P0 = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
    P3 = test_case_data.manual_poses.at("P3");
    P4 = test_case_data.manual_poses.at("P4");
    P5 = test_case_data.manual_poses.at("P5");
  };

  geometry_msgs::msg::Pose P0;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;
  geometry_msgs::msg::Pose P3;
  geometry_msgs::msg::Pose P4;
  geometry_msgs::msg::Pose P5;

  lanelet::ConstLanelets all_lanelets_;
  std::optional<lanelet2_utils::LaneletRTree> rtree_{};

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestNNSearch001, get_closest_against_P0)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P0);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2246);
    }
    {
      // rtree give same result
      const auto closest = rtree_->get_closest_lanelet(P0);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2246);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2246);
    }

    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2246);
    }
  }
}

TEST_F(TestNNSearch001, get_closest_against_P1)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P1);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2262);
    }
    {
      // rtree give same result
      const auto closest = rtree_->get_closest_lanelet(P1);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2262);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2262);
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2262);
    }
  }
}

TEST_F(TestNNSearch001, get_closest_against_P2)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P2);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2262);
    }
    {
      // rtree give same result
      const auto closest = rtree_->get_closest_lanelet(P2);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2262);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2262);
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2262);
    }
  }
}

TEST_F(TestNNSearch001, get_closest_against_P3)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P3);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2312);
    }

    {
      const auto closest = rtree_->get_closest_lanelet(P3);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2312);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2312);

      const auto closest_hard_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P3, 0.0 /* do not allow out of lane*/, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_hard_constraint.has_value());
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P3, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());

      ASSERT_EQ(closest_constraint->id(), 2312);

      const auto closest_hard_constraint = rtree_->get_closest_lanelet_within_constraint(
        P3, 0.0 /* do not allow out of lane*/, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_hard_constraint.has_value());
    }
  }
}

TEST_F(TestNNSearch001, get_closest_against_P4)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P4);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2312);
    }

    {
      const auto closest = rtree_->get_closest_lanelet(P4);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2312);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_constraint.has_value());
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P4, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_constraint.has_value()) << "found " << closest_constraint->id() << " FP";
    }
  }
}

TEST_F(TestNNSearch001, get_closest_against_P5)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P5);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2311);
    }

    {
      const auto closest = rtree_->get_closest_lanelet(P5);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2311);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P5, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_constraint.has_value());
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P5, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_FALSE(closest_constraint.has_value()) << "found " << closest_constraint->id() << " FP";
    }
  }
}

class TestNNSearch002 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "autoware_lanelet2_utils")) /
                                "test_data" / "test_nn_search_002.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet::LaneletMapConstPtr lanelet_map =
      lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);

    all_lanelets_ = lanelet_map->laneletLayer | ranges::to<std::vector>();
    rtree_.emplace(lanelet2_utils::LaneletRTree(all_lanelets_));

    P0 = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
  };

  geometry_msgs::msg::Pose P0;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;

  lanelet::ConstLanelets all_lanelets_;
  std::optional<lanelet2_utils::LaneletRTree> rtree_;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestNNSearch002, get_closest_against_P0)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P0);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2270);
    }

    {
      const auto closest = rtree_->get_closest_lanelet(P0);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2270);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2270);
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2270);
    }
  }
}

TEST_F(TestNNSearch002, get_closest_against_P1)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P1);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2270);
    }
    {
      const auto closest = rtree_->get_closest_lanelet(P1);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2270);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2270);
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2270);
    }
  }
}

TEST_F(TestNNSearch002, get_closest_against_P2)
{
  {
    {
      const auto closest = lanelet2_utils::get_closest_lanelet(all_lanelets_, P2);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2282);
    }
    {
      const auto closest = rtree_->get_closest_lanelet(P2);
      ASSERT_TRUE(closest.has_value());

      ASSERT_EQ(closest->id(), 2282);
    }
  }

  {
    // with constraint
    {
      const auto closest_constraint = lanelet2_utils::get_closest_lanelet_within_constraint(
        all_lanelets_, P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2282);
    }
    {
      // rtree give same result
      const auto closest_constraint = rtree_->get_closest_lanelet_within_constraint(
        P2, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      ASSERT_TRUE(closest_constraint.has_value());
      ASSERT_EQ(closest_constraint->id(), 2282);
    }
  }
}

class TestNNSearch003 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "autoware_lanelet2_utils")) /
                                "test_data" / "test_nn_search_003.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet_map_ptr_ = lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);

    all_lanelets_ = lanelet_map_ptr_->laneletLayer | ranges::to<std::vector>();
    rtree_.emplace(lanelet2_utils::LaneletRTree(all_lanelets_));

    P0 = test_case_data.manual_poses.at("P0");
    P1 = test_case_data.manual_poses.at("P1");
    P2 = test_case_data.manual_poses.at("P2");
    P3 = test_case_data.manual_poses.at("P3");
  };

  geometry_msgs::msg::Pose P0;
  geometry_msgs::msg::Pose P1;
  geometry_msgs::msg::Pose P2;
  geometry_msgs::msg::Pose P3;

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::ConstLanelets all_lanelets_;
  std::optional<lanelet2_utils::LaneletRTree> rtree_;

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;
};

TEST_F(TestNNSearch003, get_road_lanelets_at)
{
  {
    // for P0
    const auto & p = P0.position;
    const auto road_lanelets = lanelet2_utils::get_road_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    EXPECT_EQ(road_lanelets.size(), 3);

    ASSERT_TRUE(lanelet::utils::contains(road_lanelets, lanelet_map_ptr_->laneletLayer.get(2271)));
    ASSERT_TRUE(lanelet::utils::contains(road_lanelets, lanelet_map_ptr_->laneletLayer.get(2267)));
    ASSERT_TRUE(lanelet::utils::contains(road_lanelets, lanelet_map_ptr_->laneletLayer.get(2282)));
  }

  {
    // for P1
    const auto & p = P1.position;
    const auto shoulder_lanelets =
      lanelet2_utils::get_shoulder_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    EXPECT_EQ(shoulder_lanelets.size(), 1);

    ASSERT_TRUE(
      lanelet::utils::contains(shoulder_lanelets, lanelet_map_ptr_->laneletLayer.get(2309)));
  }

  {
    // for P2
    const auto & p = P2.position;
    const auto road_lanelets = lanelet2_utils::get_road_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    const auto shoulder_lanelets =
      lanelet2_utils::get_shoulder_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    ASSERT_TRUE(road_lanelets.empty());
    ASSERT_TRUE(shoulder_lanelets.empty());
  }

  {
    // for P3
    const auto & p = P3.position;
    const auto road_lanelets = lanelet2_utils::get_road_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    const auto shoulder_lanelets =
      lanelet2_utils::get_shoulder_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    ASSERT_TRUE(road_lanelets.empty());
    ASSERT_TRUE(shoulder_lanelets.empty());
  }
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
