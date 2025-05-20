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
/// \brief Implementation of the GMOutdoorMapsBuilder class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/GMOutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

namespace easynav
{

GMOutdoorMapsBuilder::GMOutdoorMapsBuilder(const rclcpp::NodeOptions & options)
: OutdoorMapsBuilder(options)
{
  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/map_builder/grid_map", rclcpp::QoS(1).transient_local().reliable());
}

OutdoorMapsBuilder::CallbackReturnT
GMOutdoorMapsBuilder::on_activate(const rclcpp_lifecycle::State & state)
{

  (void)state;

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

OutdoorMapsBuilder::CallbackReturnT
GMOutdoorMapsBuilder::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

void GMOutdoorMapsBuilder::cycle()
{
  if (pub_->get_subscription_count() > 0) {
    auto downsampled = PerceptionsOpsView(perceptions_).downsample(downsample_resolution_);
    auto downsampled_points = downsampled.as_points(0);

    if (downsampled_points.empty()) {
      return;
    }

    grid_map::GridMap map({"elevation"});
    map.setFrameId(perception_default_frame_);
    map.setTimestamp(perceptions_[0]->stamp.nanoseconds());

    //Get Geometry from PCL Cloud
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::min();
    
    for (const auto& pt : downsampled_points.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      
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

    map.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(center_x, center_y));
    map["elevation"].setConstant(0.0); //Initialize elevations of all cells to zero

    //Set elevation
    for (const auto& pt : downsampled_points.points) {
        grid_map::Position pos(pt.x, pt.y);
        grid_map::Index index;
        if (map.getIndex(pos, index)) {
            float& cell = map.at("elevation", index);
            if (std::isnan(cell)) 
                cell = pt.z;
            else 
                cell = std::max(cell, pt.z);
        }
    }

    auto msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_->publish(std::move(msg));

    // mark perceptions as not new after published
    for (auto & perception : perceptions_) {
      if (perception->new_data) {
        perception->new_data = false;
      }
    }
  }
}
} // namespace easynav
