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

/**
 * @class OutdoorMapsBuilderNode
 * @brief Lifecycle node that manages multiple MapsBuilder instances for outdoor map construction.
 *
 * This node subscribes to sensor data (e.g., point clouds) and manages a set of
 * MapsBuilder objects which process perceptions and publish map representations.
 * It supports ROS2 lifecycle management with proper state transitions.
 */
class OutdoorMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(OutdoorMapsBuilderNode)

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor
   * @param options NodeOptions passed to the underlying lifecycle node.
   */
  explicit OutdoorMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Destructor
  ~OutdoorMapsBuilderNode();

  /**
   * @brief Lifecycle configure callback
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle activate callback
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle deactivate callback
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle cleanup callback
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Executes a processing cycle for all registered MapsBuilder instances.
   *
   * This function is expected to be called periodically to process perceptions and
   * update maps accordingly.
   */
  void cycle();

  const Perceptions & get_perceptions() const {return perceptions_;}

private:
  /// Collection of map builders processing different map modalities.
  std::vector<std::unique_ptr<MapsBuilder>> builders_;


  /// Topic name from which sensor data (point clouds) are subscribed.
  std::string sensor_topic_;

  Perceptions perceptions_;


  /// Shared perceptions container holding fused perception data.
  std::shared_ptr<Perceptions> processed_perceptions_;

  /// Subscription to point cloud sensor data.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Callback group for subscription callbacks.
  rclcpp::CallbackGroup::SharedPtr cbg_;

  /// Resolution used for downsampling the point cloud data.
  double downsample_resolution_;

  /// Default frame ID to assign to published point cloud messages.
  std::string perception_default_frame_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDERNODE_HPP_
