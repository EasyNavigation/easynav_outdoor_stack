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

#ifndef EASYNAV_PLANNER__PCD_HPP_
#define EASYNAV_PLANNER__PCD_HPP_

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
 * @brief Simple 2D uint8_t grid using basic C++ types, with full metric conversion support.
 *
 * Supports arbitrary metric origins, allowing negative coordinates.
 */
class PCD : public MapsTypeBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Creates an empty (0x0) PCD with 1.0 meter resolution and origin (0.0, 0.0).
   */
  PCD();

  /**
   * @brief Initialize the map to new dimensions, resolution, and origin.
   *
   * @param width Number of columns.
   * @param height Number of rows.
   * @param resolution Size of each cell in meters.
   * @param origin_x Metric X coordinate corresponding to cell (0,0).
   * @param origin_y Metric Y coordinate corresponding to cell (0,0).
   * @param initial_value Value to initialize all cells.
   */
  void initialize(
    std::size_t width_, std::size_t height_, std::size_t point_step_,
    std::size_t row_step_, int initial_value_);

  /**
   * @brief Load map data and metadata from a nav_msgs::msg::OccupancyGrid message.
   *
   * This function resizes the internal grid to match the occupancy grid dimensions,
   * sets the resolution and origin, and copies the data.
   *
   * @param cloud_msg The occupancy grid message to load from.
   */
  void deep_copy(const PCD & other);

  /**
   * @brief Load map data and metadata from a nav_msgs::msg::OccupancyGrid message.
   *
   * This function resizes the internal grid to match the occupancy grid dimensions,
   * sets the resolution and origin, and copies the data.
   *
   * @param cloud_msg The occupancy grid message to load from.
   */
  void from_point_cloud(const sensor_msgs::msg::PointCloud2 & cloud_msg);

  /**
   * @brief Updates a nav_msgs::msg::OccupancyGrid message from the SimpleMap contents.
   *
   * @param cloud_msg The occupancy grid message to fill or update.
   */
  void to_point_cloud(
    sensor_msgs::msg::PointCloud2 & cloud_msg,
    pcl::PointCloud<pcl::PointXYZ> & cloud) const;

  /**
   * @brief Updates a nav_msgs::msg::OccupancyGrid message from the SimpleMap contents.
   *
   * @param cloud_msg The occupancy grid message to fill or update.
   */
  void to_point_cloud(sensor_msgs::msg::PointCloud2 & cloud_msg) const;

  /**
  * @brief Saves the map to a file, including metadata and cell data.
  * @param path Path to the output file.
  * @return true if the file was written successfully, false otherwise.
  */
  bool save_to_file(const std::string & path) const;

  /**
   * @brief Loads the map from a file, reading metadata and cell data.
   * @param path Path to the input file.
   * @return true if the file was read successfully and is valid, false otherwise.
   */
  bool load_from_file(const std::string & path);

private:
  std::size_t width_;
  std::size_t height_;
  std::size_t point_step_;
  std::size_t row_step_;
  int initial_value_;
  std::vector<sensor_msgs::msg::PointField> fields_;
  std::vector<uint8_t> data_;

  std::vector<sensor_msgs::msg::PointField> get_fields(void);
  void get_cloud(pcl::PointCloud<pcl::PointXYZ> & cloud) const;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__PCD_HPP_
