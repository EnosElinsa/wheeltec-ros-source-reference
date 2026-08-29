// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *
 */

#include "costmap_generator/object_map_utils.hpp"

#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace object_map
{
void PublishGridMap(
  const grid_map::GridMap & in_gridmap,
  const rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr in_publisher)
{
  auto message = grid_map::GridMapRosConverter::toMessage(in_gridmap);
  in_publisher->publish(*message);
}

void PublishOccupancyGrid(
  const grid_map::GridMap & in_gridmap,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr in_publisher,
  const std::string & in_layer, double in_min_value, double in_max_value, double in_height)
{
  nav_msgs::msg::OccupancyGrid message;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    in_gridmap, in_layer, in_min_value, in_max_value, message);
  message.info.origin.position.z = in_height;
  in_publisher->publish(message);
}

void fill_polygon_areas(
  grid_map::GridMap & out_grid_map, const std::vector<geometry_msgs::msg::Polygon> & in_polygons,
  const std::string & in_grid_layer_name, const float in_layer_background_value,
  const float in_fill_value)
{
  if (!out_grid_map.exists(in_grid_layer_name)) {
    out_grid_map.add(in_grid_layer_name);
  }
  out_grid_map[in_grid_layer_name].setConstant(in_layer_background_value);

  for (const auto & poly : in_polygons) {
    grid_map::Polygon grid_map_poly;
    for (const auto & p : poly.points) {
      grid_map_poly.addVertex({p.x, p.y});
    }
    for (grid_map::PolygonIterator it(out_grid_map, grid_map_poly); !it.isPastEnd(); ++it) {
      out_grid_map.at(in_grid_layer_name, *it) = in_fill_value;
    }
  }
}

}  // namespace object_map
