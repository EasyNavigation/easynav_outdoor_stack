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
/// \brief Definition of the GridMapMapsBuilder class for grid map generation from point cloud data.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDDER_HPP_

#include "easynav_outdoor_maps_builder/types/MapsBuilder.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "grid_map_msgs/msg/grid_map.hpp"

namespace easynav
{

/**
 * @class GridMapMapsBuilder
 * @brief Derived class of MapsBuilder specialized in generating grid maps from point cloud data.
 *
 * This class implements lifecycle methods and a processing cycle to convert
 * point cloud perceptions into grid map messages.
 */
class GridMapMapsBuilder : public MapsBuilder
{
public:
  /**
   * @brief Constructor.
   * @param node Pointer to the lifecycle node this builder is associated with.
   * @param shared_perceptions Shared pointer to the perceptions data used for map building.
   */
  explicit GridMapMapsBuilder(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<Perceptions> & shared_perceptions);

  /**
   * @brief Lifecycle callback called when the node is configured.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS on success, otherwise failure code.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback called when the node is activated.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS on success, otherwise failure code.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback called when the node is deactivated.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS on success, otherwise failure code.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback called when the node is cleaned up.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS on success, otherwise failure code.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Processing method called periodically to process new perception data and publish grid maps.
   */
  void cycle() override;

private:
  /// Publisher for grid map messages with lifecycle awareness.
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;

  /// Resolution used for downsampling the input point cloud data.
  double downsample_resolution_;

  /// Default coordinate frame for the published grid map messages.
  std::string perception_default_frame_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__GRIDMAPMAPSBUILDDER_HPP_
