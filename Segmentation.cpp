/*
 * Segmentation.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/board.h>

#include "Segmentation.h"

using namespace std;

PointCloud::Ptr myPlaneSegmentation(PointCloud::Ptr cloud_in){
	PointCloud::Ptr out(new PointCloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_in);
	seg.segment (*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract;

	extract.setInputCloud (cloud_in);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*out);
	return out;

}

PointCloud::Ptr PlaneSegmentationWithNormal(PointCloud::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals){
	PointCloud::Ptr out(new PointCloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentationFromNormals<PointT,pcl::Normal> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.03);
	seg.setInputCloud (cloud_in);
	seg.setInputNormals (normals);
	seg.segment (*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract;

	extract.setInputCloud (cloud_in);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*out);
	return out;

}

PointCloud::Ptr myEuclideanClusterExtraction(PointCloud::Ptr cloud_in){

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_in);
	ec.extract (cluster_indices);

	PointCloud::Ptr cloud_cluster (new PointCloud);

	std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_in->points[*pit]);
	it++;
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_in->points[*pit]);
	it++;
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_in->points[*pit]);
	it++;
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_in->points[*pit]);
	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	return cloud_cluster;

}

pcl::PointCloud<pcl::ReferenceFrame>::Ptr BoardLocalReferenceFrameEstimation(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals,PointCloud::Ptr surface,float radius){
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr out_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
	pcl::BOARDLocalReferenceFrameEstimation<PointT , pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles (true);
	rf_est.setRadiusSearch (0.015);

	rf_est.setInputCloud (cloudin);
	rf_est.setInputNormals (normals);
	rf_est.setSearchSurface (surface);
	rf_est.compute (*out_rf);

	return out_rf;
}


