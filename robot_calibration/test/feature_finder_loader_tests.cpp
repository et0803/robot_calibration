/*
 * Copyright (C) 2018-2022 Michael Ferguson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <robot_calibration/finders/loader.hpp>
#include <gtest/gtest.h>

// Horrible hack that lets us test the internals of our finders
#define private public
#define protected public

#include <robot_calibration/finders/checkerboard_finder.hpp>
#include <robot_calibration/finders/plane_finder.hpp>

TEST(FeatureFinderLoaderTests, test_feature_finder_loader)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("feature_finder_loader_tests");
  robot_calibration::FeatureFinderLoader loader;
  robot_calibration::FeatureFinderMap features;
  bool result = loader.load(node, features);

  EXPECT_EQ(true, result);
  EXPECT_EQ(static_cast<size_t>(3), features.size());

  auto checkerboard_finder =
      std::dynamic_pointer_cast<robot_calibration::CheckerboardFinder<sensor_msgs::msg::PointCloud2>>(features["checkerboard_finder"]);
  EXPECT_EQ(11, checkerboard_finder->points_x_);
  EXPECT_EQ(12, checkerboard_finder->points_y_);
  EXPECT_EQ(0.4, checkerboard_finder->square_size_);
  EXPECT_EQ(true, checkerboard_finder->output_debug_);
  EXPECT_EQ("first_frame_id", checkerboard_finder->frame_id_);
  EXPECT_EQ("first_camera", checkerboard_finder->camera_sensor_name_);
  EXPECT_EQ("first_arm", checkerboard_finder->chain_sensor_name_);

  auto checkerboard_2d_finder =
      std::dynamic_pointer_cast<robot_calibration::CheckerboardFinder<sensor_msgs::msg::Image>>(features["checkerboard_finder_2d"]);
  EXPECT_EQ(13, checkerboard_2d_finder->points_x_);
  EXPECT_EQ(14, checkerboard_2d_finder->points_y_);
  EXPECT_EQ(0.5, checkerboard_2d_finder->square_size_);
  EXPECT_EQ(false, checkerboard_2d_finder->output_debug_);
  EXPECT_EQ("second_frame_id", checkerboard_2d_finder->frame_id_);
  EXPECT_EQ("second_camera", checkerboard_2d_finder->camera_sensor_name_);
  EXPECT_EQ("second_arm", checkerboard_2d_finder->chain_sensor_name_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
