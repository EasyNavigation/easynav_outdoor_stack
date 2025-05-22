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

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_outdoor_maps_builder/types/MapsBuilder.hpp"

namespace easynav
{

/**
 * @class PointcloudMapsBuilder
 * @brief Concrete MapsBuilder implementation for processing and publishing point cloud maps.
 *
 * This class handles point cloud data, applying downsampling and publishing
 * the resulting map on a lifecycle-aware ROS2 publisher.
 */
class PointcloudMapsBuilder : public MapsBuilder
{
public:
  /**
   * @brief Constructor.
   * @param node Pointer to the lifecycle node associated with this builder.
   * @param perceptions Shared pointer to the perceptions data.
   */
  explicit PointcloudMapsBuilder(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<Perceptions> & perceptions);

  /**
   * @brief Lifecycle configure callback.
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle activate callback.
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle deactivate callback.
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle cleanup callback.
   * @param state Current lifecycle state.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Periodic processing cycle to update and publish point cloud map data.
   */
  void cycle() override;

private:
  /// Lifecycle publisher for PointCloud2 messages.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  /// Resolution used for downsampling the point cloud data.
  double downsample_resolution_;

  /// Default frame ID to assign to published point cloud messages.
  std::string perception_default_frame_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDER_HPP_
