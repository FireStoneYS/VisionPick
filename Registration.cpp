/*
 * Registration.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */


#include <opencv/highgui.h>


#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/pfhrgb.h>

#include "Registration.h"

using namespace std;

PointCloud::Ptr ICP_test(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti){

	PointCloud::Ptr out(new PointCloud);
	pcl::IterativeClosestPoint<PointT,PointT> icp;
	icp.setTransformationEpsilon(1e-3);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setEuclideanFitnessEpsilon(0.005);
	icp.setMaximumIterations(10000);
	PointCloud Final;
	icp.setInputTarget(cloud1);
	icp.setInputCloud(cloud2);
	icp.align(Final);
	Ti = icp.getFinalTransformation() * Ti;
//	cout<<Ti<<endl;
	pcl::transformPointCloud(*cloud2, *out, icp.getFinalTransformation());
//	*out += *cloud1;
	return out;
}


PointCloud::Ptr myFPFH_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti){

	PointCloud::Ptr out(new PointCloud);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud1_FPFH= compute_fpfh_feature(cloud1);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud2_FPFH= compute_fpfh_feature(cloud2);
	cout<<"compute fpfh over"<<endl;
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(cloud2);
	sac_ia.setSourceFeatures(cloud2_FPFH);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(cloud1_FPFH);
	PointCloud::Ptr align (new PointCloud);
	//  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(50); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);
	cout<<"score = "<<sac_ia.getFitnessScore()<<endl;
	Ti = sac_ia.getFinalTransformation()*Ti;
//	*out = *align+*cloud1;
	return align;




}

PointCloud::Ptr myShot_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti){

	PointCloud::Ptr out(new PointCloud);
	pcl::PointCloud<pcl::SHOT352>::Ptr cloud1_shot (new pcl::PointCloud<pcl::SHOT352> ());
	pcl::PointCloud<pcl::SHOT352>::Ptr cloud2_shot (new pcl::PointCloud<pcl::SHOT352> ());
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud1_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud2_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.setInputCloud(cloud1);
	est_normal.compute(*cloud1_normal);
	est_normal.setInputCloud(cloud2);
	est_normal.compute(*cloud2_normal);
	pcl::SHOTEstimationOMP<PointT , pcl::Normal, pcl::SHOT352> descr_est;
	descr_est.setRadiusSearch (0.02f);
	descr_est.setInputCloud (cloud1);
	descr_est.setInputNormals (cloud1_normal);
	descr_est.compute (*cloud1_shot);

	descr_est.setInputCloud (cloud2);
	descr_est.setInputNormals (cloud2_normal);
	descr_est.compute (*cloud2_shot);
	cout<<"compute shot over"<<endl;

	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::SHOT352> sac_ia;
	sac_ia.setInputSource(cloud2);
	sac_ia.setSourceFeatures(cloud2_shot);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(cloud1_shot);
	PointCloud::Ptr align (new PointCloud);
	//  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(50); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);
	cout<<"score = "<<sac_ia.getFitnessScore()<<endl;
	Ti = sac_ia.getFinalTransformation()*Ti;
//	*out = *align+*cloud1;
	return align;




}

PointCloud::Ptr ICPWithNormal(PointCloud::Ptr cloud_tgt,PointCloud::Ptr cloud_src,Eigen::Matrix4f &Ti){

	PointCloud::Ptr out(new PointCloud);
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr reg_result (new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);

	norm_est.setInputCloud (cloud_src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*cloud_src, *points_with_normals_src);

	norm_est.setInputCloud (cloud_tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*cloud_tgt, *points_with_normals_tgt);


	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>  icp;
	icp.setTransformationEpsilon(1e-3);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setEuclideanFitnessEpsilon(0.005);
	icp.setMaximumIterations(10000);
	icp.setInputTarget(points_with_normals_tgt);
	icp.setInputCloud(points_with_normals_src);
	icp.align(*reg_result);
	Ti = icp.getFinalTransformation() * Ti;
	cout<<Ti<<endl;
	pcl::transformPointCloud(*cloud_src, *out, icp.getFinalTransformation());
//	*out += *cloud1;
	return out;
}

PointCloud::Ptr myPFHRGB_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti){

	PointCloud::Ptr out(new PointCloud);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud1_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud2_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.setInputCloud(cloud1);
	est_normal.compute(*cloud1_normal);
	est_normal.setInputCloud(cloud2);
	est_normal.compute(*cloud2_normal);

	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr cloud1_PFHRGB(new pcl::PointCloud<pcl::PFHRGBSignature250>);
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr cloud2_PFHRGB(new pcl::PointCloud<pcl::PFHRGBSignature250>);
	pcl::PFHRGBEstimation<PointT, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb_est ;
	pfhrgb_est.setSearchMethod(tree);
	pfhrgb_est.setKSearch(20);
	pfhrgb_est.setInputCloud(cloud1);
	pfhrgb_est.setInputNormals(cloud1_normal);
	pfhrgb_est.compute(*cloud1_PFHRGB);
	pfhrgb_est.setInputCloud(cloud2);
	pfhrgb_est.setInputNormals(cloud2_normal);
	pfhrgb_est.compute(*cloud2_PFHRGB);


//	cout<<"compute PFHRGB over"<<endl;
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::PFHRGBSignature250> sac_ia;
	sac_ia.setInputSource(cloud2);
	sac_ia.setSourceFeatures(cloud2_PFHRGB);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(cloud1_PFHRGB);
	PointCloud::Ptr align (new PointCloud);
	//  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(50); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);
//	cout<<"score = "<<sac_ia.getFitnessScore()<<endl;
	Ti = sac_ia.getFinalTransformation()*Ti;
//	*out = *align+*cloud1;
	return align;




}
