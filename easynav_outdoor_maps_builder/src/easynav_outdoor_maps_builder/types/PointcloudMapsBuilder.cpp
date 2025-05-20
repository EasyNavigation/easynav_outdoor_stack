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
/// \brief Implementation of the PointcloudMapsBuilder class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/types/PointcloudMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

PointcloudMapsBuilder::PointcloudMapsBuilder(rclcpp_lifecycle::LifecycleNode * node)
: MapsBuilder(node)
{


  if (!node_->has_parameter("downsample_resolution")) {
    node_->declare_parameter("downsample_resolution", 1.0);
  }

  if (!node_->has_parameter("perception_default_frame")) {
    node_->declare_parameter("perception_default_frame", "map");
  }
  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder/cloud_filtered", rclcpp::QoS(1).transient_local().reliable());
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MapsBuilder::CallbackReturnT
PointcloudMapsBuilder::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;


  node_->get_parameter("downsample_resolution", downsample_resolution_);
  node_->get_parameter("perception_default_frame", perception_default_frame_);

  return CallbackReturnT::SUCCESS;
}

MapsBuilder::CallbackReturnT
PointcloudMapsBuilder::on_activate(const rclcpp_lifecycle::State & state)
{

  (void)state;

  pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

MapsBuilder::CallbackReturnT
PointcloudMapsBuilder::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

MapsBuilder::CallbackReturnT
PointcloudMapsBuilder::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  pub_.reset();
  return CallbackReturnT::SUCCESS;
}

void PointcloudMapsBuilder::cycle()
{
  if (pub_->get_subscription_count() > 0) {

    auto downsampled = PerceptionsOpsView(perceptions_).downsample(downsample_resolution_);
    auto downsampled_points = downsampled.as_points();

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
