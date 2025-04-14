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

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <lanelet2_core/Forward.h>

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lanelet2_utils
{
/**
 * @brief Get stop line regulatory element from a lanelet, optionally checking matching ID
 *
 * @param[in] lane_id lanelet ID
 * @param[in] lanelet_map_ptr lanelet map
 * @param[in] check_id_match whether to check if attribute ID matches lane ID
 * @return stop line if found
 */
std::optional<lanelet::ConstLineString3d> get_stop_line_from_deprecated_crosswalk(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr);

/**
 * @brief Get stop line from a no stopping area regulatory element in lanelets, optionally checking
 * matching ID
 *
 * @param[in] lanelets input lanelets
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief Get stop line from a no stopping area regulatory element in lanelet, optionally checking
 * matching ID
 *
 * @param[in] lanelet input lanelet
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from a detection area regulatory element in lanelets, optionally checking
 * matching ID
 *
 * @param[in] lanelet input lanelets
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief Get stop line from a detection area regulatory element in lanelet, optionally checking
 * matching ID
 *
 * @param[in] lanelet input lanelet
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelet & lanelet);

/**
 * @brief Get stop line from intersection marking regulatory element in lanelets, optionally
 * checking matching ID
 *
 * @param[in] lanelets input lanelets
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_line_from_intersection_marking(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief Get stop line from intersection marking regulatory element in lanelets, optionally
 * checking matching ID
 *
 * @param[in] lanelet input lanelet
 * @return vector of stop lines
 */
std::vector<lanelet::ConstLineString3d> get_stop_line_from_intersection_marking(
  const lanelet::ConstLanelet & lanelet);

}  // namespace autoware::lanelet2_utils

#endif  // AUTOWARE_LANELET2_UTILS__STOP_LINE_HPP_
