/*
 * Filter.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: abds
 */


#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

#include "Filter.h"

PointCloud::Ptr VoxelGridFilter(PointCloud::Ptr cloudin,float leaf){
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (cloudin);
	grid.filter (*cloudin);
	return cloudin;
}

PointCloud::Ptr Downsampling(PointCloud::Ptr cloud_in,float size){
	PointCloud::Ptr out(new PointCloud);
	PointCloud::Ptr filtered_cloud (new PointCloud);
	pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (size, size, size);
	approximate_voxel_filter.setInputCloud (cloud_in);
	approximate_voxel_filter.filter (*out);
	return out;
}

PointCloud::Ptr Passthrough1(PointCloud::Ptr cloud_in){
	PointCloud::Ptr out(new PointCloud);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud_in);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1, 1.1);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1, 0.2);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.3,0.3);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*out);
	return out;
}

PointCloud::Ptr Passthrough2(PointCloud::Ptr cloud_in){
	PointCloud::Ptr out(new PointCloud);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud_in);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1, 1.1);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.5,0.3);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.9,0.1);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*out);
	return out;
}

PointCloud::Ptr PassthroughXiyiye(PointCloud::Ptr cloud_in){
	PointCloud::Ptr out(new PointCloud);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud_in);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1,0.8);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1, 0.2);
	pass.filter (*out);
	pass.setInputCloud (out);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.3,0.3);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*out);
	return out;
}

PointCloud::Ptr RadiuSoutlierRemove(PointCloud::Ptr cloud_in){
	PointCloud::Ptr out(new PointCloud);
    pcl::RadiusOutlierRemoval<PointT> outrem;
    // build the filter
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius (10);
    // apply filter
    outrem.filter (*out);
    return out;
}

PointCloud::Ptr UniformSampling(PointCloud::Ptr cloudin,float radius){

	PointCloud::Ptr out (new PointCloud);
	pcl::UniformSampling<PointT > uniform_sampling;
	uniform_sampling.setInputCloud(cloudin);
	uniform_sampling.setRadiusSearch (radius);
	uniform_sampling.filter (*out);
	std::cout << "Model total points: " << cloudin->size () << "; Selected Keypoints: " << out->size () << std::endl;

	return out;

}

PointCloud::Ptr StatisticalOutlierRemoval(PointCloud::Ptr cloudin){

	PointCloud::Ptr out (new PointCloud);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloudin);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*out);

	return out;
}

PointCloud::Ptr UpSampling(PointCloud::Ptr cloudin){
	PointCloud::Ptr out(new PointCloud);
	pcl::MovingLeastSquares<PointT, PointT> mls;
	mls.setInputCloud (cloudin);
	mls.setSearchRadius (0.03);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (6);
	mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY);
	mls.setPointDensity(4000);
	mls.process (*out);
	return out;
}

PointCloud::Ptr Smooth(PointCloud::Ptr cloudin){
	PointCloud::Ptr out(new PointCloud);
	pcl::MovingLeastSquares<PointT, PointT> smooth;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	smooth.setComputeNormals (false);

	// Set parameters
	smooth.setInputCloud (cloudin);
	smooth.setPolynomialFit (true);
	smooth.setSearchMethod (tree);
	smooth.setSearchRadius (0.03);

	// Reconstruct
	smooth.process (*out);
	return out;
}
