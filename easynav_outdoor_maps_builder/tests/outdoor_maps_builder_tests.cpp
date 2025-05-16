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

#include <memory>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_outdoor_maps_builder/PCOutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

using namespace easynav;

class TestablePCOutdoorMapsBuilder : public PCOutdoorMapsBuilder
{
public:
  using PCOutdoorMapsBuilder::PCOutdoorMapsBuilder;

  std::vector<std::shared_ptr<Perception>> & get_perceptions() {return perceptions_;}

  using PCOutdoorMapsBuilder::pub_;
};

class PCOutdoorMapsBuilderTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_topic", "test_points");
    options.append_parameter_override("downsample_resolution", 6.0);
    options.append_parameter_override("perception_default_frame", "map");

    builder_ = std::make_shared<TestablePCOutdoorMapsBuilder>(options);

    builder_->pub_ = builder_->create_publisher<sensor_msgs::msg::PointCloud2>("output_topic", 10);
  }

  void TearDown() override
  {
    builder_.reset();
  }

  std::shared_ptr<TestablePCOutdoorMapsBuilder> builder_;
};

TEST_F(PCOutdoorMapsBuilderTest, CyclePublishesAndResetsNewDataFlag)
{
  auto & perceptions = builder_->get_perceptions();
  perceptions.clear();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.points.resize(1);
  cloud.points[0].x = 0.1f;
  cloud.points[0].y = 0.2f;
  cloud.points[0].z = 0.3f;

  auto perception = std::make_shared<Perception>();
  perception->data = cloud;
  perception->frame_id = "map";
  perception->stamp = rclcpp::Clock().now();
  perception->valid = true;
  perception->new_data = true;
  perceptions.push_back(perception);

  builder_->configure();
  builder_->pub_ = builder_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_builder/cloud_filtered", rclcpp::QoS(10));
  builder_->activate();
  auto dummy = builder_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_builder/cloud_filtered", 10,
    [](sensor_msgs::msg::PointCloud2::SharedPtr) {});

  builder_->cycle();

  EXPECT_FALSE(perception->new_data);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
