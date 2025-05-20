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

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_outdoor_maps_builder/types/MapsBuilder.hpp"

namespace easynav
{

class OutdoorMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(OutdoorMapsBuilderNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit OutdoorMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OutdoorMapsBuilderNode();


  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  void cycle();

  const Perceptions & get_perceptions() const {return perceptions_;}

protected:
  Perceptions perceptions_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::CallbackGroup::SharedPtr cbg_;

private:
  std::vector<std::unique_ptr<MapsBuilder>> builders_;
  std::string sensor_topic_;
};


} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_
