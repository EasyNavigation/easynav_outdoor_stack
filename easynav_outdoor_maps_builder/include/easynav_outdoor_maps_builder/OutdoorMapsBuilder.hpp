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
/// \brief Declaration of the OutdoorMapsBuilder base class used for generating outdoor maps from slam cloud.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace easynav
{

  /// \class OutdoorMapsBuilder
  /// \brief Abstract base class for map builders that operate in outdoor environments using point cloud data.
class OutdoorMapsBuilder : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(OutdoorMapsBuilder)

    /// \brief Alias for the lifecycle callback return type.
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /// \brief Constructor.
    /// \param options Node configuration options.
  explicit OutdoorMapsBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// \brief Destructor.
  ~OutdoorMapsBuilder();

    /// \brief Lifecycle method called when configuring the node.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

    /// \brief Lifecycle method called when activating the node.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

    /// \brief Lifecycle method called when deactivating the node.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

    /// \brief Lifecycle method called during cleanup.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

    /// \brief Lifecycle method called on shutdown.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

    /// \brief Lifecycle method called on error.
    /// \param state Current lifecycle state.
    /// \return SUCCESS or FAILURE.
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

    /// \brief Get the callback group associated with this node.
    /// \return Shared pointer to the callback group.
  rclcpp::CallbackGroup::SharedPtr get_cbg();

    /// \brief Get the current perceptions collected by the node.
    /// \return A Perceptions object containing sensor-derived data.
  const Perceptions get_perceptions() const {return perceptions_;}

    /// \brief Abstract method to be implemented by derived classes to define behavior on each processing cycle.
  virtual void cycle() = 0;

protected:
    /// \brief Subscription to the point cloud topic.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    /// \brief Container for storing perception data.
  Perceptions perceptions_;

    /// \brief Topic name for the point cloud sensor.
  std::string sensor_topic_;

    /// \brief Voxel downsampling resolution to reduce point cloud density.
  double downsample_resolution_;

    /// \brief Default frame of reference used for perception data.
  std::string perception_default_frame_;

    /// \brief Callback group to manage concurrency and executor behavior.
  rclcpp::CallbackGroup::SharedPtr cbg_;
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_
