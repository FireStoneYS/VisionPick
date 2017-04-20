/*
 * Normal.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#include <pcl/features/normal_3d_omp.h>
#include <pcl/gpu/features/features.hpp>

#include "Normal.h"

pcl::PointCloud<pcl::Normal>::Ptr NormalOMP(PointCloud::Ptr cloudin){

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::NormalEstimationOMP<PointT , pcl::Normal> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (cloudin);
	norm_est.compute (*normals);

	return normals;
}

PointCloudWithNormals::Ptr NormalEst(PointCloud::Ptr cloudin){
	PointCloudWithNormals::Ptr out (new PointCloudWithNormals);
	pcl::NormalEstimation<PointT,PointNormalT> norm_est;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);
	norm_est.setInputCloud (cloudin);
	norm_est.compute (*out);
	return out;
	std::cout<<"normal compute finished"<<std::endl;
}



