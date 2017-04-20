/*
 * Normal.h
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#ifndef NORMAL_H_
#define NORMAL_H_



#include "SaveOption.h"

pcl::PointCloud<pcl::Normal>::Ptr NormalOMP(PointCloud::Ptr cloudin);
PointCloudWithNormals::Ptr NormalEst(PointCloud::Ptr cloudin);

#endif /* NORMAL_H_ */
