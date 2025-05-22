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


#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>  // std::pair
#include <fstream>
#include <sstream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_outdoor_common/PCD.hpp"

namespace easynav
{

PCD::PCD()
: width_(0), height_(0), point_step_(1), row_step_(1), initial_value_(0), data_()
{}

void
PCD::initialize(
  std::size_t width,
  std::size_t height,
  std::size_t point_step,
  std::size_t row_step,
  int initial_value)
{
  width_ = width; 
  height_ = height; 
  point_step_ = point_step; 
  row_step_ = row_step;
  initial_value_ = initial_value; 
  fields_ = get_fields();
  data_.assign(row_step_ * height, initial_value);
}

void
PCD::deep_copy(const PCD & other)
{
  width_ = other.width_; 
  height_ = other.height_; 
  point_step_ = other.point_step_; 
  row_step_ = other.row_step_;
  initial_value_ = other.initial_value_; 
  fields_ = other.fields_;
  data_ = other.data_;
}

std::vector<sensor_msgs::msg::PointField>
PCD::get_fields()
{
  std::vector<sensor_msgs::msg::PointField> fields;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  fields.push_back(field);

  return fields;
}

void
PCD::to_point_cloud(sensor_msgs::msg::PointCloud2 & cloud_msg, pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  cloud_msg.width = static_cast<uint32_t>(width_);
  cloud_msg.height = static_cast<uint32_t>(height_);
  cloud_msg.point_step = static_cast<uint32_t>(point_step_);
  cloud_msg.row_step = static_cast<uint32_t>(row_step_);

  pcl::toROSMsg(cloud, cloud_msg);      
}

void
PCD::to_point_cloud(sensor_msgs::msg::PointCloud2 & cloud_msg) const
{
  cloud_msg.width = static_cast<uint32_t>(width_);
  cloud_msg.height = static_cast<uint32_t>(height_);
  cloud_msg.point_step = static_cast<uint32_t>(point_step_);
  cloud_msg.row_step = static_cast<uint32_t>(row_step_);
  cloud_msg.fields = fields_; 
  cloud_msg.data = data_;
}

void
PCD::from_point_cloud(const sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  initialize(
    cloud_msg.width,
    cloud_msg.height,
    cloud_msg.point_step,
    cloud_msg.row_step,
    0); 

  fields_ = cloud_msg.fields;
  data_ = cloud_msg.data;
}

void
PCD::get_cloud(pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.width = static_cast<uint32_t>(width_);
  cloud_msg.height = static_cast<uint32_t>(height_);
  cloud_msg.point_step = static_cast<uint32_t>(point_step_);
  cloud_msg.row_step = static_cast<uint32_t>(row_step_);
  cloud_msg.fields = fields_;
  cloud_msg.data = data_;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

}

bool
PCD::save_to_file(const std::string & path) const
{
  pcl::PointCloud<pcl::PointXYZ> temp_cloud;
  get_cloud(temp_cloud);
  pcl::io::savePCDFileASCII (path, temp_cloud);
  return true;
}

bool
PCD::load_from_file(const std::string & path)
{

  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, cloud) == -1) //* load the file
  {
    std::cerr << " :: Load PCD file failed :: \n";
    std::cerr << "Couldn't read file in "<< path <<" \n";
    return false;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  from_point_cloud(cloud_msg);

  return true;
}

}  // namespace easynav