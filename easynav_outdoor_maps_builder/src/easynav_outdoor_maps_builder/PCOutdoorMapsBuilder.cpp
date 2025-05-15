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
/// \brief Implementation of the PCMapBuilder class.

#include "easynav_outdoor_maps_builder/PCOutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

PCMapsBuilder::PCMapsBuilder(const rclcpp::NodeOptions & options)
: MapsBuilder(options)
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder/pcl_points", rclcpp::QoS(1).transient_local().reliable());
}

void PCMapsBuilder::build_map()
{

  if (perception_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "No perceptions available.");
    return;
  }
  Perceptions perceptions;

  perceptions.push_back(std::make_shared<Perception>(perception_));
  auto downsampled = PerceptionsOpsView(perceptions).downsample(downsample_resolution_);

  auto downsampled_points = downsampled.as_points(0);

  if (downsampled_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Downsampled cloud is empty.");
    return;
  }

  auto msg = points_to_rosmsg(downsampled_points);
  msg.header.frame_id = perception_.frame_id;
  msg.header.stamp = perception_.stamp;
  pub_->publish(msg);
}

} // namespace easynav
