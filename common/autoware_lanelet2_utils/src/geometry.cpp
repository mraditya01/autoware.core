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

#include <autoware/lanelet2_utils/geometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <iostream>
#include <vector>

namespace autoware::lanelet2_utils
{
std::optional<lanelet::ConstPoint3d> extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool is_forward)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);
  if (segment_length == 0.0) {
    return std::nullopt;
  }

  double normalized = distance / segment_length;
  double new_x, new_y, new_z;
  if (!is_forward) {
    new_x = first.x() - normalized * dx;
    new_y = first.y() - normalized * dy;
    new_z = first.z() - normalized * dz;
  } else {
    new_x = second.x() + normalized * dx;
    new_y = second.y() + normalized * dy;
    new_z = second.z() + normalized * dz;
  }
  return lanelet::ConstPoint3d(lanelet::InvalId, new_x, new_y, new_z);
}

std::optional<lanelet::ConstPoint3d> interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool from_first)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);

  if (segment_length == 0.0 || distance < 0.0 || distance > segment_length) {
    return std::nullopt;
  }

  double normalized = distance / segment_length;
  double new_x, new_y, new_z;

  if (from_first) {
    new_x = first.x() + normalized * dx;
    new_y = first.y() + normalized * dy;
    new_z = first.z() + normalized * dz;
  } else {
    new_x = second.x() - normalized * dx;
    new_y = second.y() - normalized * dy;
    new_z = second.z() - normalized * dz;
  }

  return lanelet::ConstPoint3d(lanelet::InvalId, new_x, new_y, new_z);
}

std::optional<lanelet::ConstPoint3d> interpolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool from_first)
{
  std::vector<lanelet::Point3d> points;

  if (linestring.size() < 2) {
    return std::nullopt;
  }

  double accumulated_length = 0.0;

  if (from_first) {
    for (std::size_t idx = 0; idx < linestring.size() - 1; ++idx) {
      const auto & p1 = linestring[idx];
      const auto & p2 = linestring[idx + 1];
      double segment_length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
      std::cout << p1.x() << ", " << p1.y() << ", " << p1.z() << std::endl;
      if (accumulated_length + segment_length >= distance) {
        double residue = distance - accumulated_length;
        return interpolate_point(linestring[idx], linestring[idx + 1], residue, true);
      }
      accumulated_length += segment_length;
    }
  } else {
    accumulated_length = 0.0;
    for (std::size_t idx = linestring.size() - 1; idx > 0; --idx) {
      const auto & p2 = linestring[idx];
      const auto & p1 = linestring[idx - 1];
      double segment_length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
      if (accumulated_length + segment_length >= distance) {
        double residue = distance - accumulated_length;
        return interpolate_point(linestring[idx - 1], linestring[idx], residue, false);
      }
      accumulated_length += segment_length;
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstPoint3d> extrapolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool is_forward)
{
  if (linestring.size() < 2) {
    return std::nullopt;
  }
  std::vector<lanelet::Point3d> new_points;

  if (is_forward) {
    const auto & p1 = linestring[0];
    const auto & p2 = linestring[1];
    return extrapolate_point(p1, p2, distance, false);
  } else {
    const auto & p1 = linestring[linestring.size() - 2];
    const auto & p2 = linestring.back();
    return extrapolate_point(p1, p2, distance, true);
  }
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet(
  const lanelet::ConstLanelet & lanelet, const double distance, const bool from_first)
{
  return interpolate_linestring(lanelet.centerline(), distance, from_first);
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence, double distance, bool from_first)
{
  for (const auto & llt : lanelet_sequence) {
    auto interpolated_pt = interpolate_lanelet(llt, distance, from_first);
    if (interpolated_pt.has_value()) {
      return interpolated_pt;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> concatenate_center_line(
  const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }

  std::vector<lanelet::Point3d> pts;
  pts.reserve(lanelets.size() * 10);

  for (const auto & llt : lanelets) {
    const auto & center_line = llt.centerline();
    for (const auto & cl_point : center_line) {
      if (pts.empty() || pts.back().basicPoint() != cl_point.basicPoint()) {
        pts.emplace_back(cl_point);
      }
    }
  }

  return lanelet::ConstLineString3d{lanelet::InvalId, pts};
}

}  // namespace autoware::lanelet2_utils
