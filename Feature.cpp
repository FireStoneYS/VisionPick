/*
 * Feature.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/common/centroid.h>
#include <pcl/features/crh.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/harris_2d.h>

#include "Feature.h"

pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(PointCloud::Ptr input_cloud)
{
        //法向量
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr point_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.compute(*point_normal);
	//fpfh 估计
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
//	pcl::FPFHEstimationOMP<PointT,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
//	est_fpfh.setNumberOfThreads(0); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(20);			//增大更精确
	est_fpfh.compute(*fpfh);

	return fpfh;

}

PointCloud::Ptr IssKeyPoint(PointCloud::Ptr cloudin)
{
	PointCloud::Ptr keypoints(new PointCloud);

	// ISS keypoint detector object.
	pcl::ISSKeypoint3D<PointT, PointT> detector;
	detector.setInputCloud(cloudin);
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	detector.setSearchMethod(kdtree);

	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(cloudin);

	for (size_t i = 0; i < cloudin->size(); ++i)
	{
		if (! pcl_isfinite((*cloudin)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius(6 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius(4 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads(4);

	detector.compute(*keypoints);
	return keypoints;

}


pcl::PointCloud<pcl::SHOT352>::Ptr ShotEstimationOMP(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals,PointCloud::Ptr surface,float radius){
	pcl::PointCloud<pcl::SHOT352>::Ptr out (new pcl::PointCloud<pcl::SHOT352> ());
	pcl::SHOTEstimationOMP<PointT , pcl::Normal, pcl::SHOT352> descr_est;
	descr_est.setRadiusSearch (radius);
	descr_est.setInputCloud (cloudin);
	descr_est.setInputNormals (normals);
	descr_est.setSearchSurface (surface);
	descr_est.compute (*out);
	return out;
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHEst(PointCloud::Ptr cloudin,PointCloudWithNormals::Ptr normals)
{

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<PointT, PointNormalT, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloudin);
  vfh.setInputNormals (normals);

  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);
  return vfhs;

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr CVFHEst(PointCloud::Ptr cloudin,pcl::PointCloud<pcl::Normal>::Ptr normals)
{

	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
	cvfh.setInputCloud (cloudin);
	cvfh.setInputNormals (normals);
	// alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	cvfh.setSearchMethod (tree);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step.
	cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	// Set the curvature threshold (maximum disparity between curvatures),
	// for the region segmentation step.
	cvfh.setCurvatureThreshold(1.0);
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Note: enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite.
	cvfh.setNormalizeBins(true);
	// Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	// Compute the features
	cvfh.compute (*cvfhs);
	return cvfhs;

}

//pcl::PointCloud<CRH90>::Ptr CameraRollHistogram(PointCloud::Ptr cloudin){
//
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<CRH90>::Ptr histogram(new pcl::PointCloud<CRH90>);
//
//
//	// Estimate the normals.
//	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
//	normalEstimation.setInputCloud(cloudin);
//	normalEstimation.setRadiusSearch(0.03);
//	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
//	normalEstimation.setSearchMethod(kdtree);
//	normalEstimation.compute(*normals);
//
//	// CRH estimation object.
//	pcl::CRHEstimation<PointT, pcl::Normal, CRH90> crh;
//	crh.setInputCloud(cloudin);
//	crh.setInputNormals(normals);
//	Eigen::Vector4f centroid;
//	pcl::compute3DCentroid(*cloudin, centroid);
//	crh.setCentroid(centroid);
//
//	// Compute the CRH.
//	crh.compute(*histogram);
//	return histogram;
//}

PointCloud::Ptr ISS3D(PointCloud::Ptr cloudin){
	PointCloud::Ptr out(new PointCloud);
	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	iss_detector.setSearchMethod (tree);
	iss_detector.setSalientRadius (0.02f);
	iss_detector.setNonMaxRadius (0.01f);
	iss_detector.setThreshold21 (0.975);
	iss_detector.setThreshold32 (0.975);
	iss_detector.setMinNeighbors (5);
	iss_detector.setNumberOfThreads (4);
	iss_detector.setInputCloud (cloudin);
	iss_detector.compute (*out);
	return out;
}

//PointCloud::Ptr SiftKeypoint(PointCloud::Ptr cloudin){
//
//	pcl::SIFTKeypoint<pcl::PointNormal, PointT> sift;
//	pcl::PointCloud<pcl::PointWithScale> result;
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
//	sift.setSearchMethod(tree);
//	sift.setScales(0.01f, 3, 4);
//	sift.setMinimumContrast(0.001f);
//	sift.setInputCloud(cloud_normals);
//	sift.compute(result);
//}

pcl::PointCloud<pcl::PointXYZI>::Ptr HarrisKeypoint3D(PointCloud::Ptr cloudin){
	pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::NormalEstimationOMP<PointT , pcl::Normal> norm_est;
	norm_est.setKSearch (10);
	norm_est.setInputCloud (cloudin);
	norm_est.compute (*normals);

	pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI,pcl::Normal> harris;
	harris.setInputCloud(cloudin);
	harris.setNormals(normals);
	harris.setNonMaxSupression(true);
	harris.setRadius(0.03f);
	harris.setThreshold(0.02f);
	harris.compute(*out);

	return out;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr HarrisKeypoint2D(PointCloud::Ptr cloudin){
	pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::HarrisKeypoint2D<PointT,pcl::PointXYZI> harris;
	harris.setRadiusSearch(0.03f);
	harris.setInputCloud(cloudin);
	harris.setNonMaxSupression(true);
	harris.setThreshold(0.02f);
	harris.compute(*out);

	return out;

}



