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
/// \brief Definition of the GMOutdoorMapsBuilder class for point cloud-based map generation.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__GMOutdoorMapsBuilder_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__GMOutdoorMapsBuilder_HPP_

#include "easynav_outdoor_maps_builder/OutdoorMapsBuilder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

namespace easynav
{

  /// \class GMOutdoorMapsBuilder
  /// \brief Derived class of OutdoorMapsBuilder specialized in handling point cloud data.
class GMOutdoorMapsBuilder : public OutdoorMapsBuilder
{
public:
    /// \brief Constructor.
    /// \param options Node configuration options.
  explicit GMOutdoorMapsBuilder(const rclcpp::NodeOptions & options);

    /// \brief Lifecycle method called when the node is activated.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

    /// \brief Lifecycle method called when the node is deactivated.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

    /// \brief Method executed in each processing cycle when new data is available.
  void cycle() override;

protected:
    /// \brief Lifecycle publisher for the processed point cloud data.
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__GMOutdoorMapsBuilder_HPP_
