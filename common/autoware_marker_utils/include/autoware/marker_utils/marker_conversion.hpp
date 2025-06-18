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

#ifndef AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
#define AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{
/**
 * @brief create marker array from geometry polygon based on the marker type
 * @details if marker_type is LINE_LIST, the polygon is drawn as a line list
 *          if marker_type is LINE_STRIP, the polygon is drawn as a line strip
 * @param [in] polygon geometry polygon
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the geometry polygon
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create footprint from LinearRing2d (used mainly for goal_footprint)
 * @param [in] ring LinearRing2d  to convert into marker
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @return marker array of the boost LinearRing2d
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils::LinearRing2d & ring, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from boost MultiPolygon2d (Pull over area)
 * @param [in] area_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  double z = 0.0);

/**
 * @brief return marker array from centroid of MultiPolygon2d and trajectory point
 * @param [in] multi_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils::MultiPolygon2d & multi_polygon, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from stop obstacle point
 * @param [in] stop_obstacle_point point of the stop obstacle
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the stop obstacle point
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Point & stop_obstacle_point, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker from Autoware Polygon2d
 * @param [in] polygon Autoware Polygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker of the Autoware Polygon2d
 */
visualization_msgs::msg::Marker create_autoware_geometry_marker(
  const autoware_utils::Polygon2d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, double z = 0.0);

/**
 * @brief return marker array from lanelets
 * @param [in] lanelets lanelets to create markers from
 * @param [in] z z position of the marker
 * @return marker array of the lanelets
 */
visualization_msgs::msg::MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, double z);

/**
 * @brief return marker array from lanelets
 * @details This function creates a marker array from lanelets, draw the lanelets either as triangle
 * marker or boundary as marker.
 * @param [in] lanelets lanelets to create markers from
 * @param [in] color color of the marker
 * @param [in] ns namespace of the marker
 * @return marker array of the lanelets
 */
visualization_msgs::msg::MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & color,
  const std::string & ns = "");

/**
 * @brief create marker array from predicted object
 * @details This function creates a marker array from a PredictedObjects object
 * (initial_pose_with_covariance), draw the vehicle based on initial pose.
 * @param [in] objects PredictedObjects object
 * @param [in] stamp current time
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & stamp,
  const std::string & ns, const int32_t id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief add predicted objects ego vehicle pose marker array from PredictedObjects to existing
 * marker array
 * @details This function creates a marker array from a PredictedObjects object (predicted_path),
 * draw the vehicle based on predicted path ego vehicle pose.
 * @param [in] objects PredictedObjects object
 * @param [in] ego_pose pose of the ego vehicle
 */
visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const geometry_msgs::msg::Pose & ego_pose);

/**
 * @brief create predicted path marker array from PredictedPath
 * @param [in] predicted_path PredictedPath object
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] ns namespace for the marker
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the predicted path
 */
visualization_msgs::msg::MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create a vehicle trajectory point marker array object
 * @param [in] mpt_traj trajectory points to create markers from
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] sampling_num sampling number to reduce the number of markers
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t sampling_num);

// TODO(Giovanni) Find a solution for this
/**
 * @brief create a vehicle trajectory point marker array object
 * @details Temporary function to calculate the arc length of the path since
 * original function from "autoware/behavior_path_planner_common/utils/path_utils.hpp"
 * does not support any type beside tier4_planning_msgs::msg::PathWithLaneId.
 * @param [in] path PathWithLaneId object to calculate arc length from
 * @return vector of arc lengths
 */
template <typename PathWithLaneIdT>
std::vector<double> calc_path_arc_length_array(const PathWithLaneIdT & path)
{
  std::vector<double> out;
  if (path.points.empty()) return out;

  out.reserve(path.points.size());
  double sum = 0.0;
  out.push_back(sum);

  for (size_t i = 1; i < path.points.size(); ++i) {
    sum += autoware_utils::calc_distance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief create marker array from lanelet polygon (CompoundPolygon3d or ConstPolygon3d)
 * @param [in] polygon lanelet polygon
 * @param [in] header header of the marker
 * @param [in] ns namespace
 * @param [in] color color of the marker
 * @return marker array of the lanelet polygon
 */
template <typename PolygonT>
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const PolygonT & polygon, const std_msgs::msg::Header & header, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = autoware_utils::create_default_marker(
    header.frame_id, header.stamp, ns, 0, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.1, 0.0, 0.0), color);
  for (const auto & p : polygon) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    marker.points.push_back(pt);
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

/**
 * @brief create marker array from predicted object (tier4 msg or internal planning msg)
 * @param [in] path PathWithLaneId object
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] with_text if true, add text to the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
template <typename PathWithLaneIdT>
visualization_msgs::msg::MarkerArray create_path_with_lane_id_marker_array(
  const PathWithLaneIdT & path, const std::string & ns, const int32_t lane_id,
  const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text)
{
  auto uid = lane_id << (sizeof(int32_t) * 8 / 2);
  int32_t idx = 0;
  int32_t i = 0;
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
    "map", now, ns, static_cast<int32_t>(uid), visualization_msgs::msg::Marker::ARROW, scale,
    color);

  for (const auto & p : path.points) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.pose = p.point.pose;

    if (
      std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) == p.lane_ids.end() && !with_text) {
      marker.color = autoware_utils::create_marker_color(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
    if (i % 10 == 0 && with_text) {
      const auto arclength = calc_path_arc_length_array(path);
      visualization_msgs::msg::Marker marker_text = autoware_utils::create_default_marker(
        "map", now, ns, 0L, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        autoware_utils::create_marker_scale(0.2, 0.1, 0.3),
        autoware_utils::create_marker_color(1, 1, 1, 0.999));
      marker_text.id = uid + i++;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }
  return msg;
}

}  // namespace autoware::experimental::marker_utils

#endif  // AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
