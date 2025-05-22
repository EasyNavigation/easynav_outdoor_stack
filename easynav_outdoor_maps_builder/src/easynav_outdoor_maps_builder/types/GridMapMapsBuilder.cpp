// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the GridMapMapsBuilder class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/types/GridMapMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

namespace easynav
{
GridMapMapsBuilder::GridMapMapsBuilder(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<Perceptions> & shared_perceptions)
: MapsBuilder(node, shared_perceptions)
{

  if (!node_->has_parameter("gridmap.downsample_resolution")) {
    node_->declare_parameter("gridmap.downsample_resolution", 1.0);
  }

  if (!node_->has_parameter("gridmap.perception_default_frame")) {
    node_->declare_parameter("gridmap.perception_default_frame", "map");
  }

  pub_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "map_builder/grid_map", rclcpp::QoS(1).transient_local().reliable());
}

MapsBuilder::CallbackReturnT
GridMapMapsBuilder::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;


  node_->get_parameter("gridmap.downsample_resolution", downsample_resolution_);
  node_->get_parameter("gridmap.perception_default_frame", perception_default_frame_);

  return CallbackReturnT::SUCCESS;
}

MapsBuilder::CallbackReturnT
GridMapMapsBuilder::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}


MapsBuilder::CallbackReturnT
GridMapMapsBuilder::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

MapsBuilder::CallbackReturnT
GridMapMapsBuilder::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  pub_.reset();
  return CallbackReturnT::SUCCESS;
}

void GridMapMapsBuilder::cycle()
{
  if (pub_->get_subscription_count() > 0) {
    auto & shared_perceptions = *perceptions_;
    auto downsampled = PerceptionsOpsView(shared_perceptions).downsample(downsample_resolution_);
    auto downsampled_points = downsampled.as_points();

    if (downsampled_points.empty()) {
      return;
    }

    grid_map::GridMap map({"elevation"});
    map.setFrameId(perception_default_frame_);
    map.setTimestamp(shared_perceptions[0]->stamp.nanoseconds());

      // Get Geometry from PCL Cloud
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min();

    for (const auto & pt : downsampled_points.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
    }

    float resolution = 1;
    float length_x = max_x - min_x;
    float length_y = max_y - min_y;
    float center_x = (max_x + min_x) / 2.0;
    float center_y = (max_y + min_y) / 2.0;

    map.setGeometry(grid_map::Length(length_x, length_y), resolution,
                      grid_map::Position(center_x, center_y));
    map["elevation"].setConstant(0.0);   // Initialize elevations of all cells to zero

      // Set elevation
    for (const auto & pt : downsampled_points.points) {
      grid_map::Position pos(pt.x, pt.y);
      grid_map::Index index;
      if (map.getIndex(pos, index)) {
        float & cell = map.at("elevation", index);
        if (std::isnan(cell)) {
          cell = pt.z;
        } else {
          cell = (cell + pt.z) / 2.0;
        }
      }
    }

    auto msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_->publish(std::move(msg));
  }
}
} // namespace easynav
