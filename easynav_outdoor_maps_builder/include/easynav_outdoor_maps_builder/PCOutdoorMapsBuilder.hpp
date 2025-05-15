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
/// \brief Declaration of PCMapBuilder class.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__PCOUTDOORMAPSBUILDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__PCOUTDOORMAPSBUILDER_HPP_

#include "easynav_outdoor_maps_builder/OutdoorMapsBuilder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{
  /**
   * @brief Concrete implementation of MapsBuilder using PointCloud2 data.
   *
   * This class extends MapsBuilder to build maps specifically from point cloud data.
   */
class PCMapsBuilder : public MapsBuilder
{
public:
    /**
     * @brief Constructor for PCMapsBuilder.
     * @param options Node options used for ROS2 node initialization.
     */
  explicit PCMapsBuilder(const rclcpp::NodeOptions & options);

    /**
     * @brief Implements the abstract build_map method to build the map from point cloud data.
     */
  void build_map() override;

private:
    /// Publisher to publish processed PointCloud2 map data.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__PCOUTDOORMAPSBUILDER_HPP_
