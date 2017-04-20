/*
 * Feature.h
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#ifndef FEATURE_H_
#define FEATURE_H_


#include "SaveOption.h"


pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<PointT>::Ptr input_cloud);
pcl::PointCloud<pcl::SHOT352>::Ptr ShotEstimationOMP(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals,PointCloud::Ptr surface,float radius);
pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHEst(PointCloud::Ptr cloudin,PointCloudWithNormals::Ptr normals);
pcl::PointCloud<pcl::VFHSignature308>::Ptr CVFHEst(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals);
//pcl::PointCloud<CRH90>::Ptr CameraRollHistogram(PointCloud::Ptr cloudin);
PointCloud::Ptr ISS3D(PointCloud::Ptr cloudin);
PointCloud::Ptr IssKeyPoint(PointCloud::Ptr cloudin);
pcl::PointCloud<pcl::PointXYZI>::Ptr HarrisKeypoint3D(PointCloud::Ptr cloudin);
pcl::PointCloud<pcl::PointXYZI>::Ptr HarrisKeypoint2D(PointCloud::Ptr cloudin);


#endif /* FEATURE_H_ */
