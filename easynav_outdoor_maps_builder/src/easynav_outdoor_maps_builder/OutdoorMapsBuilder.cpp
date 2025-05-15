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
/// \brief Implementation of the MapBuilder class.

#include "easynav_outdoor_maps_builder/OutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

MapsBuilder::MapsBuilder(const rclcpp::NodeOptions & options)
: rclcpp::Node("maps_builder_node", options)
{
  if (!this->has_parameter("sensor_topic")) {
    this->declare_parameter("sensor_topic", "points");
  }
  this->get_parameter("sensor_topic", sensor_topic_);

  if (!this->has_parameter("downsample_resolution")) {
    this->declare_parameter("downsample_resolution", 1.0);
  }
  this->get_parameter("downsample_resolution", downsample_resolution_);

  if (!this->has_parameter("perception_default_frame")) {
    this->declare_parameter("perception_default_frame", "map");
  }
  this->get_parameter("perception_default_frame", perception_default_frame_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sensor_topic_, qos,
        std::bind(&MapsBuilder::perception_callback, this, std::placeholders::_1));
}

void MapsBuilder::perception_callback(const typename sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, perception_.data);
  perception_.frame_id = msg->header.frame_id;
  perception_.stamp = msg->header.stamp;
  perception_.valid = true;
  build_map();
}
} // namespace easynav
