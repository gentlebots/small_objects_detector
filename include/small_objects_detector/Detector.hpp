/*
 Copyright (c) 2023 José Miguel Guerrero Hernández

 Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     https://creativecommons.org/licenses/by-sa/4.0/

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

#ifndef INCLUDE_SMALL_OBJECTS_DETECTOR__DETECTOR_HPP_
#define INCLUDE_SMALL_OBJECTS_DETECTOR__DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/ransac.h"

namespace small_objects_detector
{

using std::placeholders::_1;

class Detector : public rclcpp::Node
{
public:
  Detector();

private:
  pcl::PointCloud<pcl::PointXYZRGB> processing(
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
  const;

  void topic_callback_pointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_;

  uint32_t plane_size_;
  float downsampling_;
};

} // namespace small_objects_detector

#endif  // INCLUDE_SMALL_OBJECTS_DETECTOR__DETECTOR_HPP_
