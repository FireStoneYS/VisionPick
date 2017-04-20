/*
 * Filter.h
 *
 *  Created on: Apr 7, 2017
 *      Author: abds
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "SaveOption.h"

PointCloud::Ptr Downsampling(PointCloud::Ptr cloud_in,float size);
PointCloud::Ptr Passthrough1(PointCloud::Ptr cloud_in);
PointCloud::Ptr Passthrough2(PointCloud::Ptr cloud_in);
PointCloud::Ptr PassthroughXiyiye(PointCloud::Ptr cloud_in);
PointCloud::Ptr RadiuSoutlierRemove(PointCloud::Ptr cloud_in);
PointCloud::Ptr UniformSampling(PointCloud::Ptr cloudin,float radius);
PointCloud::Ptr VoxelGridFilter(PointCloud::Ptr cloudin,float leaf);
PointCloud::Ptr StatisticalOutlierRemoval(PointCloud::Ptr cloudin);
PointCloud::Ptr UpSampling(PointCloud::Ptr cloudin);
PointCloud::Ptr Smooth(PointCloud::Ptr cloudin);

#endif /* FILTER_H_ */
