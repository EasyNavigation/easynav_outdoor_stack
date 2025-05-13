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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the GpsVffLocalizer class.

#include <expected>
#include "easynav_gps_vff_localizer/GpsVffLocalizer.hpp"

namespace easynav
{

std::expected<void, std::string> GpsVffLocalizer::on_initialize()
{
  // Initialize the odometry message
  odom_.header.stamp = get_node()->now();
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";

  // Create subscriber to GPS data
  gps_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/robot/gps/fix", rclcpp::SensorDataQoS().reliable(),
    std::bind(&GpsVffLocalizer::gps_callback, this, std::placeholders::_1));

  // Create static broadcaster
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node());

  // Create static transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = get_node()->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  static_broadcaster_->sendTransform(transform);
  return {};
}

void GpsVffLocalizer::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gps_msg_ = std::move(*msg);
}

nav_msgs::msg::Odometry GpsVffLocalizer::get_odom()
{
  return odom_;
}

void GpsVffLocalizer::update(const NavState & nav_state)
{
  update_rt(nav_state);
}

void GpsVffLocalizer::update_rt(const NavState & nav_state)
{
  odom_.header.stamp = nav_state.timestamp;
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";

  // Convert GPS coordinates to UTM
  double utm_x, utm_y;
  std::string utm_zone;
  robot_localization::navsat_conversions::LLtoUTM(
    gps_msg_.latitude, gps_msg_.longitude, utm_y, utm_x, utm_zone);

  if (origin_utm_ == geometry_msgs::msg::Point() &&
    gps_msg_ != sensor_msgs::msg::NavSatFix())
  {
    // Get first UTM coordinates
    origin_utm_.x = utm_x;
    origin_utm_.y = utm_y;
  }

  // Get XY cartesian coordinates respect to the origin
  odom_.pose.pose.position.x = utm_x - origin_utm_.x;
  odom_.pose.pose.position.y = utm_y - origin_utm_.y;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::GpsVffLocalizer, easynav::LocalizerMethodBase)
