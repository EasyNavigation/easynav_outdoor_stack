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
/// \brief Declaration of MapBuilder class.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/types/Perceptions.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
namespace easynav
{

  /**
   * @brief Abstract base class for building maps from sensor data.
   *
   * This class inherits from rclcpp::Node and subscribes to a PointCloud2 topic
   * to build maps via the build_map() method, which must be implemented by derived classes.
   */

class MapsBuilder : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for MapsBuilder.
     * @param options Configuration options for the ROS2 node.
     */

  explicit MapsBuilder(const rclcpp::NodeOptions & options);

    /**
     * @brief Default virtual destructor.
     */

  virtual ~MapsBuilder() = default;

    /**
     * @brief Pure virtual method to build the map.
     * Must be implemented in derived classes.
     */
  virtual void build_map() = 0;

protected:
    /// Subscription to the PointCloud2 sensor topic.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    /// Object representing processed perceptions.
  Perception perception_;
    /// Name of the sensor topic from which data is received.
  std::string sensor_topic_;
    /// Resolution used for downsampling the point cloud.
  double downsample_resolution_;

    /// Default reference frame for the perception data.
  std::string perception_default_frame_;
    /**
     * @brief Callback function that processes received PointCloud2 messages.
     * @param msg Shared pointer to the received point cloud message.
     */
  void perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

} // namespace easynav

#endif // EASYNAV_OUTDOOR_MAPS_BUILDER__OUTDOORMAPSBUILDER_HPP_
