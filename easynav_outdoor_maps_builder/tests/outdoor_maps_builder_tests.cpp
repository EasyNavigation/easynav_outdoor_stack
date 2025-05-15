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
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "easynav_outdoor_maps_builder/PCOutdoorMapsBuilder.hpp"
#include "easynav_common/types/Perceptions.hpp"

using namespace easynav;

// Clase derivada para acceder a miembros protegidos
class TestablePCMapsBuilder : public PCMapsBuilder
{
public:
  using MapsBuilder::perception_callback;
  using PCMapsBuilder::PCMapsBuilder;

  Perception & getPerception() {return perception_;}
};

class PCMapsBuilderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override("sensor_topic", "test_points");
    node_options_.append_parameter_override("downsample_resolution", 0.1);
    node_options_.append_parameter_override("perception_default_frame", "map");

    builder_ = std::make_shared<TestablePCMapsBuilder>(node_options_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::NodeOptions node_options_;
  std::shared_ptr<TestablePCMapsBuilder> builder_;
};

TEST_F(PCMapsBuilderTest, ParametersAreInitialized)
{
  EXPECT_EQ(builder_->get_parameter("sensor_topic").as_string(), "test_points");
  EXPECT_DOUBLE_EQ(builder_->get_parameter("downsample_resolution").as_double(), 0.1);
  EXPECT_EQ(builder_->get_parameter("perception_default_frame").as_string(), "map");
}

TEST_F(PCMapsBuilderTest, BuildMapWithEmptyPerception)
{
  builder_->getPerception().data.clear();
  testing::internal::CaptureStderr();
  builder_->build_map();
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_NE(output.find("No perceptions available."), std::string::npos);
}

TEST_F(PCMapsBuilderTest, PerceptionCallbackFillsPerception)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = rclcpp::Time(123456);
  cloud_msg.height = 1;
  cloud_msg.width = 1;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);

  auto it_x = sensor_msgs::PointCloud2Iterator<float>(cloud_msg, "x");
  auto it_y = sensor_msgs::PointCloud2Iterator<float>(cloud_msg, "y");
  auto it_z = sensor_msgs::PointCloud2Iterator<float>(cloud_msg, "z");

  *it_x = 1.0f;
  *it_y = 2.0f;
  *it_z = 3.0f;

  builder_->perception_callback(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg));

  const auto & cloud = builder_->getPerception().data;
  EXPECT_EQ(cloud.size(), 1);
  EXPECT_FLOAT_EQ(cloud.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud.points[0].y, 2.0f);
  EXPECT_FLOAT_EQ(cloud.points[0].z, 3.0f);

  EXPECT_EQ(builder_->getPerception().frame_id, "map");
  EXPECT_EQ(builder_->getPerception().stamp.nanoseconds(), 123456);
}

TEST_F(PCMapsBuilderTest, TestBuildMapPublishesMessage)
{
  Perception p;
  p.frame_id = "map";
  p.stamp = rclcpp::Time(0);
  p.valid = true;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  cloud.points[0].x = 1.0f;
  cloud.points[0].y = 2.0f;
  cloud.points[0].z = 0.0f;

  cloud.points[1].x = 3.0f;
  cloud.points[1].y = 4.0f;
  cloud.points[1].z = 0.0f;

  cloud.points[2].x = -1.0f;
  cloud.points[2].y = -1.0f;
  cloud.points[2].z = 0.0f;

  p.data = cloud;
  builder_->getPerception() = p;

  builder_->build_map();

  SUCCEED() << "build_map() successfully executed";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
