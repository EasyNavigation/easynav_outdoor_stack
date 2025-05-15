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
/// \brief Implementation of the MapBuilder class.

#include "rclcpp/rclcpp.hpp"
#include "easynav_outdoor_maps_builder/PCOutdoorMapsBuilder.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<easynav::PCMapsBuilder>(rclcpp::NodeOptions());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
