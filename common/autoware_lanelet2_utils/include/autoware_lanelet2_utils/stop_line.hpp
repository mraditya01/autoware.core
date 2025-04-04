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

#ifndef AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
#define AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// Autoware & lanelet2 includes
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <autoware/motion_utils/trajectory/trajectory.hpp>  // findFirstNearestIndexWithSoftConstraints
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::lanelet2_utils
{
using Point2d = autoware::universe_utils::Point2d;
using LineString2d = autoware::universe_utils::Line2d;

/**
 * @brief Get the extended 2D geometry of a given stop line
 (originally from lanelet::autoware::DetectionArea & detection_area input)
 *
 * @param[in] stop_line original stop line
 * @param[in] extend_length length to extend the stop line
 * @return extended 2D line
 */
LineString2d get_stop_line_geometry2d(
  const lanelet::ConstLineString3d & stop_line, double extend_length);

/**
 * @brief Extract stop lines associated with a stop sign ID from lanelets
 *
 * @param[in] lanelets input lanelets
 * @param[in] stop_sign_id traffic sign ID for stop sign
 * @return vector of matching stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id);

/**
 * @brief Get particular stop line regulatory element from road_marking, optionally checking matching ID
 * @param[in] lane_id lanelet ID
 * @param[in] lanelet_map_ptr lanelet map
 * @param[in] attribute_name optional attribute to match
 * @param[in] check_id_match whether to check if attribute ID matches lane ID
 * @return stop line if found
 */
std::optional<lanelet::ConstLineString3d> get_stop_line_from_road_marking(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name, bool check_id_match = true);

/**
 * @brief Get stop line from a no stopping area regulatory element in lanelet
 *
 * @param[in] lanelets input lanelets
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief Get stop line from a detection area regulatory element in lanelet
 *
 * @param[in] lanelets input lanelets
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelets & lanelets);

}  // namespace autoware::lanelet2_utils

#endif  // AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
