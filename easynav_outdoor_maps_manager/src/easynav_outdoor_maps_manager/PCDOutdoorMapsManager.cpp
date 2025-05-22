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
/// \brief Implementation of the PCDOutdoorMapsManager class.

#include <stdexcept>

#include "easynav_outdoor_maps_manager/PCDOutdoorMapsManager.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{

std::expected<void, std::string>
PCDOutdoorMapsManager::on_initialize()
{
  static_map_ = std::make_shared<PCD>();
  dynamic_map_ = std::make_shared<PCD>();

  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file, map_topic_in;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);
  node->declare_parameter(plugin_name + ".map_topic_in", map_topic_in);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);
  node->get_parameter(plugin_name + ".map_topic_in", map_topic_in);

  if (package_name != "" && map_path_file != "") {
    std::string pkgpath;
    try {
      pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      map_path_ = pkgpath + "/" + map_path_file;
    } catch(ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (!static_map_->load_from_file(map_path_)) {
      return std::unexpected("File [" + map_path_ + "] not found");
    }
  }

  static_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    node->get_name() + std::string("/") + plugin_name + "/static_map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    node->get_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  incoming_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    node->get_name() + std::string("/") + plugin_name + map_topic_in,
    rclcpp::QoS(1).transient_local().reliable(),
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

      static_map_->from_point_cloud(*msg);
      dynamic_map_->from_point_cloud(*msg);

      static_map_->to_point_cloud(static_map_msg_);
      static_map_msg_.header.frame_id = "map";
      static_map_msg_.header.stamp = this->get_node()->now();

      static_map_pub_->publish(static_map_msg_);
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      if (!static_map_->save_to_file(map_path_)) {
        response->success = false;
        response->message = "Failed to save map to: " + map_path_;
      } else {
        response->success = true;
        response->message = "Map successfully saved to: " + map_path_;
      }
    });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node,
    true);

  return {};
}

std::map<std::string, std::shared_ptr<MapsTypeBase>>
PCDOutdoorMapsManager::get_maps()
{
  std::map<std::string, std::shared_ptr<MapsTypeBase>> ret;
  ret["PCD.static"] = static_map_;
  ret["PCD.dynamic"] = dynamic_map_;

  return ret;
}

void
PCDOutdoorMapsManager::set_static_map(std::shared_ptr<MapsTypeBase> new_map)
{
  auto typed_ptr = std::dynamic_pointer_cast<PCD>(new_map);
  if (!typed_ptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "PCDOutdoorMapsManager::set_static_map: pointer is not a Point Cloud Map");
  } else {
    static_map_ = typed_ptr;
  }
}

void
PCDOutdoorMapsManager::set_dynamic_map(std::shared_ptr<MapsTypeBase> new_map)
{
  auto typed_ptr = std::dynamic_pointer_cast<PCD>(new_map);
  if (!typed_ptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "PCDOutdoorMapsManager::set_dynamic_map: pointer is not a Point Cloud Map");
  } else {
    dynamic_map_ = typed_ptr;
  }
}

void
PCDOutdoorMapsManager::update(const NavState & nav_state)
{
  dynamic_map_->deep_copy(*static_map_);

  std::cerr << "3*" << std::endl;
  auto fused = PerceptionsOpsView(nav_state.perceptions)
    .fuse("map")->filter({NAN, NAN, NAN}, {NAN, NAN, NAN})
    .as_points();
  // auto fused = (PerceptionsOpsView(nav_state.perceptions).fuse("map")).as_points();


  dynamic_map_->to_point_cloud(dynamic_map_msg_, fused);

  dynamic_map_msg_.header.frame_id = "map";
  dynamic_map_msg_.header.stamp = get_node()->now();
  dynamic_map_pub_->publish(dynamic_map_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::PCDOutdoorMapsManager, easynav::MapsManagerBase)
