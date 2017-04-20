/*
 * Registration.h
 *
 *  Created on: Apr 9, 2017
 *      Author: abds
 */

#ifndef REGISTRATION_H_
#define REGISTRATION_H_


#include "SaveOption.h"
#include "Feature.h"

PointCloud::Ptr ICP_test(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti);
PointCloud::Ptr myFPFH_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti);
PointCloud::Ptr myShot_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti);
PointCloud::Ptr ICPWithNormal(PointCloud::Ptr cloud_tgt,PointCloud::Ptr cloud_src,Eigen::Matrix4f &Ti);
PointCloud::Ptr myPFHRGB_Registration(PointCloud::Ptr cloud1,PointCloud::Ptr cloud2,Eigen::Matrix4f &Ti);


#endif /* REGISTRATION_H_ */
