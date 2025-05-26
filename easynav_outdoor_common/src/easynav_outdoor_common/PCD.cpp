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
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_outdoor_common/PCD.hpp"

namespace easynav
{

PCD::PCD()
: width_(0), height_(0), point_step_(1), row_step_(1), data_()
{}

void
PCD::initialize(
  std::size_t width,
  std::size_t height,
  std::size_t point_step,
  std::size_t row_step)
{
  width_ = width;
  height_ = height;
  point_step_ = point_step;
  row_step_ = row_step;
  fields_ = get_fields();
  data_.assign(row_step_ * height, 0);
}

void
PCD::deep_copy(const PCD & other)
{
  width_ = other.width_;
  height_ = other.height_;
  point_step_ = other.point_step_;
  row_step_ = other.row_step_;
  fields_ = other.fields_;
  data_ = other.data_;
}

std::vector<sensor_msgs::msg::PointField>
PCD::get_fields() const
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
PCD::refresh(
  sensor_msgs::msg::PointCloud2 & cloud_msg,
  pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  pcl::PCLPointCloud2 cloud1_in, cloud2_in, cloud_out;
  std::vector<pcl::PCLPointField> fields;
  pcl_conversions::toPCL(fields_, fields);
  cloud1_in.width = static_cast<uint32_t>(width_);
  cloud1_in.height = static_cast<uint32_t>(height_);
  cloud1_in.point_step = static_cast<uint32_t>(point_step_);
  cloud1_in.row_step = static_cast<uint32_t>(row_step_);
  cloud1_in.fields = fields;
  cloud1_in.data = data_;

  pcl::toPCLPointCloud2(cloud, cloud2_in);

  pcl::concatenate(cloud1_in, cloud2_in, cloud_out);

  pcl::PointCloud<pcl::PointXYZ> out;
  pcl::fromPCLPointCloud2(cloud_out, out);
  pcl::toROSMsg(out, cloud_msg);
}

void
PCD::to_point_cloud(sensor_msgs::msg::PointCloud2 & cloud_msg) const
{
  std::cerr << "Getting Data from msg\n";
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
    cloud_msg.row_step);

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
  std::cerr << " Path to save map: " << path << "\n";
  pcl::PointCloud<pcl::PointXYZ> temp_cloud;
  get_cloud(temp_cloud);
  pcl::io::savePCDFileASCII (path, temp_cloud);
  return true;
}

bool
PCD::load_from_file(const std::string & path)
{

  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) == -1) { //* load the file
    std::cerr << " :: Load PCD file failed :: \n";
    std::cerr << "Couldn't read file in " << path << " \n";
    return false;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  from_point_cloud(cloud_msg);

  return true;
}

void
PCD::show(void) const
{
  std::cerr << ":: PCD values :: \n";
  std::cerr << "width: " << width_ << "\n";
  std::cerr << "height: " << height_ << "\n";
  std::cerr << "point step: " << point_step_ << "\n";
  std::cerr << "row step: " << row_step_ << "\n";
  std::cerr << "fields lenght: " << fields_.size() << "\n";
  std::cerr << "data lenght: " << data_.size() << "\n";
}

}  // namespace easynav
