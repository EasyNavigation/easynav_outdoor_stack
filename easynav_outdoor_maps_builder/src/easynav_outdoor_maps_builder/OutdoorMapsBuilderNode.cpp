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


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/OutdoorMapsBuilderNode.hpp"
#include "easynav_outdoor_maps_builder/types/MapsBuilder.hpp"
#include "easynav_outdoor_maps_builder/types/PointcloudMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

OutdoorMapsBuilderNode::OutdoorMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("outdoor_maps_builder_node", options)
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!has_parameter("sensor_topic")) {
    declare_parameter("sensor_topic", "points");
  }

  if (!has_parameter("map_types")) {
    declare_parameter("map_types", std::vector<std::string>{});
  }

}

OutdoorMapsBuilderNode::~OutdoorMapsBuilderNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
OutdoorMapsBuilderNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter("sensor_topic", sensor_topic_);


  std::vector<std::string> map_types;

  get_parameter("map_types", map_types);

  for (const auto & map_type : map_types) {
    if (map_type == "pcl") {
      RCLCPP_INFO(this->get_logger(), "Adding map builder type: '%s'", map_type.c_str());
      builders_.push_back(std::make_unique<PointcloudMapsBuilder>(this));
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown map type: '%s'", map_type.c_str());
    }
  }

  for (auto & builder : builders_) {
    if (builder) {
      auto ret = builder->on_configure(state);
      if (ret != CallbackReturnT::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to configure a builder");
        return ret;
      }
    }
  }


  auto perception_entry = std::make_shared<Perception>();
  perception_entry->data.points.clear();
  perception_entry->data.clear();
  perception_entry->frame_id = "";
  perception_entry->stamp = now();
  perception_entry->valid = false;
  perception_entry->new_data = true;

  perceptions_.push_back(perception_entry);

  perception_entry->subscription = create_typed_subscription<sensor_msgs::msg::PointCloud2>(
        *this, sensor_topic_, perception_entry, cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OutdoorMapsBuilderNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  for (auto & builder : builders_) {
    if (builder) {
      auto ret = builder->on_activate(state);
      if (ret != CallbackReturnT::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to activate a builder");
        return ret;
      }
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OutdoorMapsBuilderNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & builder : builders_) {
    if (builder) {
      auto ret = builder->on_deactivate(state);
      if (ret != CallbackReturnT::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to deactivate a builder");
        return ret;
      }
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OutdoorMapsBuilderNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & builder : builders_) {
    if (builder) {
      auto ret = builder->on_cleanup(state);
      if (ret != CallbackReturnT::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to cleanup a builder");
        return ret;
      }
    }
  }

  builders_.clear();
  perceptions_.clear();

  return CallbackReturnT::SUCCESS;
}

void OutdoorMapsBuilderNode::cycle()
{
  for (auto & builder : builders_) {
    if (builder) {
      builder->cycle();
    }
  }
}


} // namespace easynav
