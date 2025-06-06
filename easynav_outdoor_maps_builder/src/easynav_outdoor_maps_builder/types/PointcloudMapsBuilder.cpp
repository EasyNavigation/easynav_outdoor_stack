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

PointcloudMapsBuilder::PointcloudMapsBuilder(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<PerceptionsOpsView> & processed_perceptions)
: MapsBuilder(node, processed_perceptions)
{

  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder/cloud_filtered", rclcpp::QoS(1).transient_local().reliable());
}

MapsBuilder::CallbackReturnT
PointcloudMapsBuilder::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

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

    auto & data_perception = processed_perceptions_->get_perceptions()[0];  // supported just one perception source

    if (processed_perceptions_->as_points().empty()) {
      return;
    }
    auto msg = points_to_rosmsg(processed_perceptions_->as_points());
    msg.header.frame_id = data_perception->frame_id;
    msg.header.stamp = data_perception->stamp;
    pub_->publish(msg);
  }
}
} // namespace easynav
