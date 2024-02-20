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

#include "small_objects_detector/Detector.hpp"

namespace small_objects_detector
{

Detector::Detector()
: Node("small_objects_detector")
{
  this->declare_parameter("plane_size", 5000);
  this->get_parameter("plane_size", plane_size_);
  this->declare_parameter("downsampling", 0.01);
  this->get_parameter("downsampling", downsampling_);

  subscription_pointcloud_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud_in", 1,
    std::bind(&Detector::topic_callback_pointcloud, this, _1));

  publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::SensorDataQoS().reliable());
}

void Detector::topic_callback_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg)
{
  // Set "check_subscription_count" to False in the launch file if you want to process it always
  if (publisher_pointcloud_->get_subscription_count() > 0) {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud);

    // Image and PointCloud processing
    pcl::PointCloud<pcl::PointXYZRGB> output = processing(pointcloud);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 out_pointcloud;
    pcl::toROSMsg(output, out_pointcloud);
    out_pointcloud.header = pointcloud_msg->header;

    // Publish the data
    publisher_pointcloud_->publish(out_pointcloud);
  }
}

pcl::PointCloud<pcl::PointXYZRGB> Detector::processing(
  const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(in_pointcloud, *cloud_filtered);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(downsampling_, downsampling_, downsampling_);
  sor.filter(*cloud_filtered_ptr_inliers);

  // Extract indices
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(
    new pcl::PointCloud<pcl::PointXYZRGB>);
  // PointCloud with final indices
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGB>);

  do {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered_ptr_inliers);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a plane model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers - plane
    extract.setInputCloud(cloud_filtered_ptr_inliers);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    // std::cerr << "PointCloud representing the plane component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object - points without plane
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered_ptr_inliers.swap(cloud_f);
  } while (cloud_p->size() > plane_size_);

  //----------------------------------------------------
  // StatisticalOutlierRemoval Filter
  //----------------------------------------------------
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
  // pass the input point cloud to the processing module
  f.setInputCloud (cloud_f);
  // set some parameters 
  f.setMeanK (50); 
  f.setStddevMulThresh (3.0);
  // get the output point cloud
  f.filter (*cloud_f);
  // std::cerr << "PointCloud inliners: " << cloud_filtered_ptr_inliers->width * cloud_filtered_ptr_inliers->height << " data points." << std::endl;

  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud = *cloud_f;

  return out_pointcloud;
}

} // namespace small_objects_detector
