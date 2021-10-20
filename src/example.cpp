/* Copyright (c) Michigan Technological University (MTU)
 * All rights reserved.
 * This project is distributed under the MIT License.
 * A copy should have been included with the source distribution.
 * If not, you can obtain a copy at https://opensource.org/licenses/MIT
 *
 * Authors: Akhil Kurup <amkurup@mtu.edu>
 *
 * A bare-bones example to subscribe to point clouds and de-noise
 *  them using the Dynamic statistical outlier removal filter (dsor)
 */

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dsor.hpp"

int k_;
float std_, range_mul_;

ros::Publisher cloud_pub;

// change this to the point type you are working with
using PointT = pcl::PointXYZ;

// Point Cloud callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  // initialize pcl objects
  pcl::PointCloud<PointT>::Ptr input_cloud(
      new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *input_cloud);

  // DSOR filter
  pcl::PointCloud<PointT>::Ptr cloud_filtered_dsor(
      new pcl::PointCloud<PointT>);
  cloud_filtered_dsor = dsor(input_cloud, k_, std_, range_mul_, false);

  // publish filtered cloud
  sensor_msgs::PointCloud2 filtered_cloud;
  pcl::toROSMsg(*cloud_filtered_dsor, filtered_cloud);
  filtered_cloud.header.frame_id = input->header.frame_id;
  filtered_cloud.header.stamp = ros::Time::now();
  cloud_pub.publish(filtered_cloud);
}


int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "dynamic_statistical_outlier_removal");
  ros::NodeHandle nh("~");
  // read params from launch file
  nh.getParam("k", k_);
  nh.getParam("std", std_);
  nh.getParam("range_mul", range_mul_);

  // subscribers & publishers
  ros::Subscriber cloud_sub = nh.subscribe("cloud_in", 1, cloud_cb);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

  ros::spin();
}
