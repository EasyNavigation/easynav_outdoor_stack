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
/// \brief Declaration of the PCD type.

#ifndef EASYNAV_OUTDOOR_MAPS_MANAGER__PCD_HPP_
#define EASYNAV_OUTDOOR_MAPS_MANAGER__PCD_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "easynav_common/types/MapTypeBase.hpp"

namespace easynav
{

/**
 * @class PCD
 * @brief Point Cloud representation using basic C++ types.
 */
class PCD : public MapsTypeBase
{
public:
  /**
   * @brief Default constructor.
   */
  PCD();

  /**
   * @brief Initialize the map with characteristics of Point Cloud 2.
   *
   * @param width Number of columns.
   * @param height Number of rows.
   * @param point_step Number of elements per point.
   * @param row_step Number of elements per row.
   */
  void initialize(
    std::size_t width_, std::size_t height_, std::size_t point_step_,
    std::size_t row_step_);

  /**
   * @brief Load map data from a PCD object.
   *
   * This function get elements from another PCD object to copy data.
   *
   * @param other PCD object source of data.
   */
  void deep_copy(const PCD & other);

  /**
   * @brief Load map data from sensor_msgs::msg::PointCloud2 message.
   *
   * This function read data from a Point Cloud 2 msg to set values
   * in PCD objects.
   *
   * @param cloud_msg The Point Cloud 2 message to load from.
   */
  void from_point_cloud(const sensor_msgs::msg::PointCloud2 & cloud_msg);

  /**
   * @brief Updates a sensor_msgs::msg::PointCloud2 message from the sensors
   * values into Easynav.
   *
   * @param cloud_msg The Point Cloud 2 message to be configured.
   * @param cloud Point Cloud with updated values from sensors.
   */
  void refresh(
    sensor_msgs::msg::PointCloud2 & cloud_msg,
    pcl::PointCloud<pcl::PointXYZ> & cloud) const;

  /**
   * @brief Updates a sensor_msgs::msg::PointCloud2 message from the PCD
   * contents.
   *
   * @param cloud_msg The Point Cloud 2 message to fill or update.
   */
  void to_point_cloud(sensor_msgs::msg::PointCloud2 & cloud_msg) const;

  /**
  * @brief Saves the map to a file.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  bool save_to_file(const std::string & path) const;

  /**
   * @brief Loads the map from a file.
   * @param path Path to the input file.
   * @return true if the file was read successfully and is valid, false otherwise.
   */
  bool load_from_file(const std::string & path);

  void show(void) const;

private:
  std::size_t width_;
  std::size_t height_;
  std::size_t point_step_;
  std::size_t row_step_;
  std::vector<sensor_msgs::msg::PointField> fields_;
  std::vector<uint8_t> data_;

  std::vector<sensor_msgs::msg::PointField> get_fields(void) const;
  void get_cloud(pcl::PointCloud<pcl::PointXYZ> & cloud) const;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_MANAGER__PCD_HPP_
