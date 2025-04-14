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

#include "autoware_lanelet2_utils/stop_line.hpp"
#include "map_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware
{
class TestGetStopLineFromMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapPtr lanelet_map_intersection_ptr_{nullptr};
  lanelet::LaneletMapPtr lanelet_map_regulatory_elements_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_intersection_ptr_ =
      load_mgrs_coordinate_map<lanelet::LaneletMapPtr>(intersection_crossing_map_path.string());

    const auto regulatory_elements_map_path =
      sample_map_dir / "intersection" / "regulatory_elements.osm";

    lanelet_map_regulatory_elements_ptr_ =
      load_mgrs_coordinate_map<lanelet::LaneletMapPtr>(regulatory_elements_map_path.string());
  }
};

// Test 1: deprecated crosswalk stop line retrieval.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDeprecatedCrosswalk)
{
  lanelet::Id valid_lane_id = 2273;
  auto stopline_opt = lanelet2_utils::get_stop_line_from_deprecated_crosswalk(
    valid_lane_id, lanelet_map_intersection_ptr_);
  if (stopline_opt.has_value()) {
    std::cout << "Deprecated Crosswalk Stop line ID: " << stopline_opt->id() << std::endl;
  }
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found on lane " << valid_lane_id << " using deprecated crosswalk.";
  ASSERT_EQ(stopline_opt->id(), 1993) << "Unexpected stop line ID for deprecated crosswalk.";
}

// Test 2: get_stop_lines_from_no_stopping_area without target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromNoStoppingArea_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt = lanelet2_utils::get_stop_lines_from_no_stopping_area(lanelets);
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for NoStoppingArea without target_id.";
  lanelet::Id expected_stop_line_id = 208;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id << ") not found in NoStoppingArea.";
}

// Test 3: get_stop_lines_from_no_stopping_area with valid target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromNoStoppingArea_TargetFound)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_lines_from_no_stopping_area(lanelets, lanelet::Id(208));
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for NoStoppingArea with target_id 208.";
  EXPECT_EQ(stopline_opt->id(), 208)
    << "Stop line ID does not match target 208 in NoStoppingArea.";
}

// Test 4: get_stop_lines_from_no_stopping_area with non-existent target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromNoStoppingArea_TargetNotFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_lines_from_no_stopping_area(lanelets, lanelet::Id(9999));
  EXPECT_FALSE(stopline_opt.has_value())
    << "Stop line found for NoStoppingArea when none was expected with target_id 9999.";
}

// Test 5: get_stop_lines_from_detection_area without target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDetectionArea_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_lines_from_detection_area(lanelets);
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for DetectionArea without target_id.";
  lanelet::Id expected_stop_line_id = 209;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id << ") not found in DetectionArea.";
}

// Test 6: get_stop_lines_from_detection_area with valid target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDetectionArea_TargetFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_lines_from_detection_area(lanelets, lanelet::Id(209));
  ASSERT_TRUE(stopline_opt.has_value())
      << "No stop line found for DetectionArea with target_id 209.";
  EXPECT_EQ(stopline_opt->id(), 209)
      << "Stop line ID does not match target 209 in DetectionArea.";
}

// Test 7: get_stop_lines_from_detection_area with non-existent target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDetectionArea_TargetNotFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_lines_from_detection_area(lanelets, lanelet::Id(9999));
  EXPECT_FALSE(stopline_opt.has_value())
      << "Stop line unexpectedly found for DetectionArea with target_id 9999.";
}

// Test 8: get_stop_line_from_intersection_marking without target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromIntersectionMarking_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_intersection_marking(lanelets);
  ASSERT_TRUE(stopline_opt.has_value())
      << "No stop line found for IntersectionMarking without target_id.";
  lanelet::Id expected_stop_line_id = 198;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
      << "Expected stop line ID (" << expected_stop_line_id
      << ") not found in IntersectionMarking.";
}

// Test 9: get_stop_line_from_intersection_marking with valid target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromIntersectionMarking_TargetFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_intersection_marking(lanelets, lanelet::Id(198));
  ASSERT_TRUE(stopline_opt.has_value())
      << "No stop line found for IntersectionMarking with target_id 198.";
  EXPECT_EQ(stopline_opt->id(), 198)
      << "Stop line ID does not match target 198 in IntersectionMarking.";
}

// Test 10: get_stop_line_from_intersection_marking with non-existent target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromIntersectionMarking_TargetNotFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt =
    lanelet2_utils::get_stop_line_from_intersection_marking(lanelets, lanelet::Id(9999));
  EXPECT_FALSE(stopline_opt.has_value())
      << "Stop line unexpectedly found for IntersectionMarking with target_id 9999.";
}

// Test 11: Empty lanelet container returns no stop line for get_stop_lines_from_no_stopping_area.
TEST_F(TestGetStopLineFromMap, EmptyLaneletContainer_NoStoppingArea)
{
  lanelet::ConstLanelets empty_lanelets;
  auto stopline_opt = lanelet2_utils::get_stop_lines_from_no_stopping_area(empty_lanelets);
  EXPECT_FALSE(stopline_opt.has_value())
      << "Stop line found in empty lanelet container for NoStoppingArea.";
}

// Test 12: Single lanelet overload usage for get_stop_lines_from_detection_area.
TEST_F(TestGetStopLineFromMap, SingleLanelet_DetectionArea)
{
  if (!lanelet_map_regulatory_elements_ptr_->laneletLayer.empty()) {
    auto lanelet = *lanelet_map_regulatory_elements_ptr_->laneletLayer.begin();
    auto stopline_opt = lanelet2_utils::get_stop_lines_from_detection_area(lanelet);
    EXPECT_TRUE(stopline_opt.has_value())
        << "No stop line found for single lanelet in DetectionArea overload.";
  } else {
    FAIL() << "No lanelets available in regulatory elements map.";
  }
}

// Test 13: get_stop_lines_from_stop_sign without target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromStopSign_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());
  lanelet::Id expected_stop_line_id = 206;
  auto stopline_opt = lanelet2_utils::get_stop_lines_from_stop_sign(lanelets);

  ASSERT_TRUE(stopline_opt.has_value())
      << "No stop line found for get_stop_lines_from_stop_sign without target_id.";
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
      << "Expected stop line ID (" << expected_stop_line_id 
      << ") not found for get_stop_lines_from_stop_sign without target_id.";
}

// Test 14: get_stop_lines_from_stop_sign with valid target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromStopSign_TargetFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt = lanelet2_utils::get_stop_lines_from_stop_sign(lanelets, lanelet::Id(206));
  ASSERT_TRUE(stopline_opt.has_value())
      << "No stop line found for get_stop_lines_from_stop_sign with target_id 206.";
  
  EXPECT_EQ(stopline_opt->id(), 206)
      << "Stop line ID does not match target 206 for get_stop_lines_from_stop_sign.";
}

// Test 15: get_stop_lines_from_stop_sign with non-existent target_id.
TEST_F(TestGetStopLineFromMap, GetStopLineFromStopSign_TargetNotFound)
{
  const lanelet::ConstLanelets lanelets(
      lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
      lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stopline_opt = lanelet2_utils::get_stop_lines_from_stop_sign(lanelets, lanelet::Id(9999));
  EXPECT_FALSE(stopline_opt.has_value())
      << "Stop line unexpectedly found for get_stop_lines_from_stop_sign with target_id 9999.";
}
}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
