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
/// \brief Implementation of the PCOutdoorMapsBuilder class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/PCOutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

PCOutdoorMapsBuilder::PCOutdoorMapsBuilder(const rclcpp::NodeOptions & options)
: OutdoorMapsBuilder(options)
{
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_builder/cloud_filtered", rclcpp::QoS(1).transient_local().reliable());
}

OutdoorMapsBuilder::CallbackReturnT
PCOutdoorMapsBuilder::on_activate(const rclcpp_lifecycle::State & state)
{

  (void)state;

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

OutdoorMapsBuilder::CallbackReturnT
PCOutdoorMapsBuilder::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

void PCOutdoorMapsBuilder::cycle()
{
  if (pub_->get_subscription_count() > 0) {

    auto downsampled = PerceptionsOpsView(perceptions_).downsample(downsample_resolution_);
    auto downsampled_points = downsampled.as_points(0);

    if (downsampled_points.empty()) {
      return;
    }

    auto msg = points_to_rosmsg(downsampled_points);
    msg.header.frame_id = perception_default_frame_;
    msg.header.stamp = perceptions_[0]->stamp;
    pub_->publish(msg);

      // mark perceptions as not new after published
    for (auto & perception : perceptions_) {
      if (perception->new_data) {
        perception->new_data = false;
      }
    }
  }
}
} // namespace easynav
