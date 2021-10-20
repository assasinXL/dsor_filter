/* Copyright (c) Michigan Technological University (MTU)
 * All rights reserved.
 * This project is distributed under the MIT License.
 * A copy should have been included with the source distribution.
 * If not, you can obtain a copy at https://opensource.org/licenses/MIT
 *
 * Authors: Akhil Kurup <amkurup@mtu.edu>
 *
 * Dynamic range-based statistical outlier removal filter (dsor)
 *  - can be used to de-noise lidar pointclouds in the presence of snow
 */
#ifndef INCLUDE_DSOR_HPP_
#define INCLUDE_DSOR_HPP_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// change this to the point type you are working with
using PointT = pcl::PointXYZ;

// dynamic stastical outlier filter
pcl::PointCloud<PointT>::Ptr dsor(pcl::PointCloud<PointT>::Ptr &input_cloud,
                                  int mean_k, float std_mul, float range_mul,
                                  bool negative) {
  // initialize cloud to hold filtered values
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr negative_cloud(new pcl::PointCloud<PointT>);

  // initialize kd search tree
  pcl::KdTreeFLANN<PointT> kd_tree;
  kd_tree.setInputCloud(input_cloud);

  // Allocate enough space to hold the results
  std::vector<int> pointIdxNKNSearch(mean_k);
  std::vector<float> pointNKNSquaredDistance(mean_k);
  std::vector<float> mean_distances;

  // Go over all the points and check which doesn't have enough neighbors
  // perform filtering
  for (pcl::PointCloud<PointT>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    // k nearest search
    kd_tree.nearestKSearch(*it, mean_k, pointIdxNKNSearch,
                           pointNKNSquaredDistance);

    // calculate mean distance
    double dist_sum = 0;
    for (int j = 1; j < mean_k; ++j) {
      dist_sum += sqrt(pointNKNSquaredDistance[j]);
    }
    mean_distances.push_back(static_cast<float>(dist_sum / (mean_k - 1)));
  }

  // Estimate the mean and the standard deviation of the distance vector
  double sum = 0, sq_sum = 0;
  for (size_t i = 0; i < mean_distances.size(); ++i) {
    sum += mean_distances[i];
    sq_sum += mean_distances[i] * mean_distances[i];
  }
  double mean = sum / static_cast<double>(mean_distances.size());
  double variance =
      (sq_sum - sum * sum / static_cast<double>(mean_distances.size())) /
      (static_cast<double>(mean_distances.size()) - 1);
  double stddev = sqrt(variance);
  ROS_DEBUG_STREAM("mean: " << mean << " var: " << variance);

  // calculate distance threshold (PCL sor implementation)
  double distance_threshold = (mean + std_mul * stddev);
  // iterate through vector
  int i = 0;
  for (pcl::PointCloud<PointT>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    // calculate distance of every point from the sensor
    float range = sqrt(pow(it->x, 2) + pow(it->y, 2) + pow(it->z, 2));
    // dynamic threshold: as a point is farther away from the sensor,
    // the threshold increases
    double dynamic_threshold = distance_threshold * range_mul * range;
    ROS_DEBUG_STREAM("dynamic threshold: " << dynamic_threshold);

    // a distance lower than the threshold is an inlier
    if (mean_distances[i] < dynamic_threshold) {
      filtered_cloud->push_back(*it);
    } else {
      negative_cloud->push_back(*it);
    }
    // update iterator
    i++;
  }

  if (negative) {
    return (negative_cloud);
  } else {
    return (filtered_cloud);
  }
}

#endif  // INCLUDE_DSOR_HPP_
