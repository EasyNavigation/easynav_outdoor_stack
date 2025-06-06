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

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__MAPSBUILDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__MAPSBUILDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

  /**
   * @class MapsBuilder
   * @brief Abstract base class for map builders in the EasyNav outdoor mapping system.
   *
   * This class defines the interface and lifecycle methods that all concrete map builder
   * implementations must provide. It manages access to the shared perceptions data
   * and the lifecycle node.
   */
class MapsBuilder
{
public:
    /// Alias for the lifecycle callback return type.
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief Constructor.
     * @param node Pointer to the lifecycle node associated with this builder.
     * @param shared_perceptions Shared pointer to the perceptions data used for map construction.
     */
  explicit MapsBuilder(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<PerceptionsOpsView> & shared_perceptions)
  : node_(node), processed_perceptions_(shared_perceptions) {}

    /// Virtual destructor.
  virtual ~MapsBuilder() = default;

    /**
     * @brief Called during the configure lifecycle transition.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) = 0;

    /**
     * @brief Called during the activate lifecycle transition.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  virtual CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) = 0;

    /**
     * @brief Called during the deactivate lifecycle transition.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  virtual CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) = 0;

    /**
     * @brief Called during the cleanup lifecycle transition.
     * @param state Current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  virtual CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) = 0;

    /**
     * @brief Periodic processing method to update map data.
     *
     * Called regularly (e.g., in a main loop) to perform map building operations.
     */
  virtual void cycle() = 0;

    /**
     * @brief Set the shared processed perceptions data.
     * @param pp Shared pointer to updated PerceptionsOpsView.
     */
  void set_processed_perceptions(const std::shared_ptr<PerceptionsOpsView> & pp)
  {
    processed_perceptions_ = pp;
  }

protected:
    /// Pointer to the lifecycle node this builder belongs to.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    /// Shared perceptions data used by this builder.
  std::shared_ptr<PerceptionsOpsView> processed_perceptions_;
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__MAPSBUILDER_HPP_
