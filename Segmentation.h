/*
 * Segmentation.h
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#include "SaveOption.h"


PointCloud::Ptr myPlaneSegmentation(PointCloud::Ptr cloud_in);
PointCloud::Ptr myEuclideanClusterExtraction(PointCloud::Ptr cloud_in);
pcl::PointCloud<pcl::ReferenceFrame>::Ptr BoardLocalReferenceFrameEstimation(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals,PointCloud::Ptr surface,float radius);
PointCloud::Ptr PlaneSegmentationWithNormal(PointCloud::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals);



