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

#include "autoware/lanelet2_utils/geometry.hpp"

#include "map_loader.hpp"

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware
{

class ExtrapolatedLaneletTest : public ::testing::Test
{
protected:
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ =
      load_mgrs_coordinate_map<lanelet::LaneletMapPtr>(intersection_crossing_map_path.string());
  }
};

// Test 1: forward extrapolation
TEST(ExtrapolatedPointTest, ForwardExtrapolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  double distance = 5.0;

  auto interpolated_pt = lanelet2_utils::extrapolate_point(p1, p2, distance, true);
  ASSERT_TRUE(interpolated_pt.has_value());
  auto point = *interpolated_pt;

  EXPECT_NEAR(point.x(), 15.0, 1e-6);
  EXPECT_NEAR(point.y(), 0.0, 1e-6);
  EXPECT_NEAR(point.z(), 0.0, 1e-6);
}

// Test 2: Backward extrapolation
TEST(ExtrapolatedPointTest, BackwardExtrapolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  double distance = 5.0;

  auto interpolated_pt = lanelet2_utils::extrapolate_point(p1, p2, distance, false);
  ASSERT_TRUE(interpolated_pt.has_value());
  auto point = *interpolated_pt;

  EXPECT_NEAR(point.x(), -5.0, 1e-6);
  EXPECT_NEAR(point.y(), 0.0, 1e-6);
  EXPECT_NEAR(point.z(), 0.0, 1e-6);
}

// Test 3: Zero distance extrapolation
TEST(ExtrapolatedPointTest, ZeroDistanceReturnsOrigin)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;

  auto interpolated_pt = lanelet2_utils::extrapolate_point(p1, p2, distance, false);
  ASSERT_TRUE(interpolated_pt.has_value());
  auto point = *interpolated_pt;

  EXPECT_NEAR(point.x(), p1.x(), 1e-6);
  EXPECT_NEAR(point.y(), p1.y(), 1e-6);
  EXPECT_NEAR(point.z(), p1.z(), 1e-6);
}

// Test 4: Zero distance interpolation
TEST(InterpolatePointTest, ZeroDistanceReturnsFirstOrLast)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;

  auto interpolated_pt_first = lanelet2_utils::interpolate_point(p1, p2, distance, true);
  auto interpolated_pt_last = lanelet2_utils::interpolate_point(p1, p2, distance, false);
  ASSERT_TRUE(interpolated_pt_first.has_value());
  ASSERT_TRUE(interpolated_pt_last.has_value());

  EXPECT_NEAR(interpolated_pt_first->x(), p1.x(), 1e-6);
  EXPECT_NEAR(interpolated_pt_last->x(), p2.x(), 1e-6);
}

// Test 5: Interpolation at exact segment length
TEST(InterpolatePointTest, AtSegmentEnd)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  auto interpolated_pt_forward = lanelet2_utils::interpolate_point(p1, p2, segment_length, true);
  auto interpolated_pt_backward = lanelet2_utils::interpolate_point(p1, p2, segment_length, false);
  ASSERT_TRUE(interpolated_pt_forward.has_value());
  ASSERT_TRUE(interpolated_pt_backward.has_value());

  EXPECT_NEAR(interpolated_pt_forward->x(), p2.x(), 1e-6);
  EXPECT_NEAR(interpolated_pt_backward->x(), p1.x(), 1e-6);
}

// Test 6: out‑of‑bounds interpolation (returns nullopt)
TEST(InterpolatePointTest, OutOfBoundsDistanceReturnsNullopt)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  auto interpolated_pt_pos = lanelet2_utils::interpolate_point(p1, p2, segment_length + 1.0, true);
  auto interpolated_pt_neg = lanelet2_utils::interpolate_point(p1, p2, -1.0, true);
  EXPECT_FALSE(interpolated_pt_pos.has_value());
  EXPECT_FALSE(interpolated_pt_neg.has_value());
}

// Test 7: interpolate_linestring from first
TEST(LineStringTest, InterpolatePointFromFirst)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double s = 15.0;
  bool from_first = true;
  auto interpolated_pt = lanelet2_utils::interpolate_linestring(line, s, from_first);

  ASSERT_TRUE(interpolated_pt.has_value());
  EXPECT_NEAR(interpolated_pt->x(), 15.0, 1e-6);
  EXPECT_NEAR(interpolated_pt->y(), 0.0, 1e-6);
  EXPECT_NEAR(interpolated_pt->z(), 0.0, 1e-6);
}

// Test 8: interpolate_linestring from last
TEST(LineStringTest, InterpolatePointFromLast)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double s = 5.0;
  bool from_first = false;
  auto interpolated_pt = lanelet2_utils::interpolate_linestring(line, s, from_first);

  ASSERT_TRUE(interpolated_pt.has_value());
  EXPECT_NEAR(interpolated_pt->x(), 15.0, 1e-6);
  EXPECT_NEAR(interpolated_pt->y(), 0.0, 1e-6);
  EXPECT_NEAR(interpolated_pt->z(), 0.0, 1e-6);
}

// Test 9: extrapolate_linestring forward
TEST(LineStringTest, ExtrapolatePointForward)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(2, 10.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(3, 20.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};

  auto interpolated_pt = lanelet2_utils::extrapolate_linestring(line, 5.0, false);
  ASSERT_TRUE(interpolated_pt.has_value());

  const auto & p = *interpolated_pt;
  EXPECT_NEAR(p.x(), 25.0, 1e-6);
  EXPECT_NEAR(p.y(), 0.0, 1e-6);
  EXPECT_NEAR(p.z(), 0.0, 1e-6);
}

// Test 10: extrapolate_linestring backward
TEST(LineStringTest, ExtrapolatePointBackward)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(2, 10.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(3, 20.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};

  auto interpolated_pt = lanelet2_utils::extrapolate_linestring(line, 5.0, true);
  ASSERT_TRUE(interpolated_pt.has_value());

  const auto & p = *interpolated_pt;
  EXPECT_NEAR(p.x(), -5.0, 1e-6);
  EXPECT_NEAR(p.y(), 0.0, 1e-6);
  EXPECT_NEAR(p.z(), 0.0, 1e-6);
}

// Test 11: extrapolate_linestring zero from first (0 distance)
TEST(LineStringTest, ExtrapolateZeroDistanceFromStart)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 5.0, 7.0, 9.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(2, 6.0, 8.0, 9.5)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};

  auto interpolated_pt = lanelet2_utils::extrapolate_linestring(line, 0.0, true);
  ASSERT_TRUE(interpolated_pt.has_value());

  const auto & p = *interpolated_pt;
  EXPECT_NEAR(p.x(), 5.0, 1e-6);
  EXPECT_NEAR(p.y(), 7.0, 1e-6);
  EXPECT_NEAR(p.z(), 9.0, 1e-6);
}

// Test 12: interpolate_lanelet test from map
TEST_F(ExtrapolatedLaneletTest, InterpolateLanelet)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(2287);
  auto opt_pt = lanelet2_utils::interpolate_lanelet(ll, 3.0, true);
  ASSERT_TRUE(opt_pt.has_value());
  EXPECT_NEAR(opt_pt->x(), 164.269030, 1e-6);
  EXPECT_NEAR(opt_pt->y(), 181.097588, 1e-6);
  EXPECT_NEAR(opt_pt->z(), 100.000000, 1e-6);
}

// Test 13: interpolate_lanelet_sequence test from map
TEST_F(ExtrapolatedLaneletTest, InterpolateLaneletSequence)
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(3);
  for (const auto & id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  auto opt_pt = lanelet2_utils::interpolate_lanelet_sequence(lanelets, 3.0, true);
  ASSERT_TRUE(opt_pt.has_value());
  EXPECT_NEAR(opt_pt->x(), 164.269030, 1e-6);
  EXPECT_NEAR(opt_pt->y(), 181.097588, 1e-6);
  EXPECT_NEAR(opt_pt->z(), 100.000000, 1e-6);
}

// Test 14: concatenate_center_line empty input
TEST(ConcatenateCenterLineTest, EmptyInputReturnsNullopt)
{
  lanelet::ConstLanelets empty_seq;
  auto opt_ls = lanelet2_utils::concatenate_center_line(empty_seq);
  EXPECT_FALSE(opt_ls.has_value());
}

// Test 15: concatenate_center_line map
TEST_F(ExtrapolatedLaneletTest, ConcatenateCenterlinesSequence)
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(3);
  for (auto id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }

  auto opt_ls = lanelet2_utils::concatenate_center_line(lanelets);

  ASSERT_TRUE(opt_ls.has_value());
  const auto & ls = *opt_ls;

  const auto & first_expected = lanelets.front().centerline().front().basicPoint();
  EXPECT_NEAR(ls.front().x(), first_expected.x(), 1e-6);
  EXPECT_NEAR(ls.front().y(), first_expected.y(), 1e-6);
  EXPECT_NEAR(ls.front().z(), first_expected.z(), 1e-6);
  const auto & last_expected = lanelets.back().centerline().back().basicPoint();
  EXPECT_NEAR(ls.back().x(), last_expected.x(), 1e-6);
  EXPECT_NEAR(ls.back().y(), last_expected.y(), 1e-6);
  EXPECT_NEAR(ls.back().z(), last_expected.z(), 1e-6);

  for (size_t i = 1; i < ls.size(); ++i) {
    EXPECT_FALSE(ls[i].basicPoint() == ls[i - 1].basicPoint());
  }
}

}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
