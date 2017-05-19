/*
 * Calibration.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: abds
 */
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <algorithm>

#include "Calibration.h"
#include "SaveOption.h"
#include "Init.h"

using namespace std;

bool Point2fSort(cv::Point2f & p1,cv::Point2f & p2){
	return p1.x<p2.x;
}
void TablePosition(){

	char key =0;
	xn::Context            	context;
	xn::ImageGenerator     	mimage_generator;
	xn::DepthGenerator     	mdepth_generator;
	NiOption np(context,mimage_generator,mdepth_generator);

	cv::Mat rgb8u( 480,640,CV_8UC3);
	cv::Mat bgr;
	cv::Mat gray,gray_filter;
	cv::Mat threshimg;
	cv::Mat canny;

	int thresh=50;
	int cannythresh=160;
	cv::namedWindow("TrackBar");
	std::vector<cv::Point2f> image_coordinate;

	np.SingleInit();
	while( (key!=27) && !( context.WaitNoneUpdateAll( ))  ){

		// 9b. get the image map
		mimage_generator.GetMetaData(np.imageMD);
		memcpy(rgb8u.data,np.imageMD.Data(),640*480*3);
		cv::cvtColor(rgb8u,bgr,cv::COLOR_RGB2BGR);
		cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);

		cv::Mat bgr_copy;
		bgr_copy = bgr.clone();
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::createTrackbar("thresh : ","TrackBar",&thresh,255);
		cv::createTrackbar("cannythresh : ","TrackBar",&cannythresh,255);
		cv::GaussianBlur(gray,gray_filter,cv::Size(5,5),0);
		cv::Canny(gray_filter,canny,cannythresh,cannythresh*2);
		cv::threshold(canny,threshimg,thresh,255,cv::THRESH_BINARY);
		cv::findContours(threshimg, contours,hierarchy, cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(0, 0));

		std::vector<int> number;
		for(size_t i =0 ;i < hierarchy.size();i++){
			int k =i ;
			int c =0;
			while (hierarchy[k][2] != -1 ){
				k = hierarchy[k][2];
				c++;
			}
			if(c >= 5){
				number.push_back(i);
			}
		}
		for(size_t i = 0;i < number.size() ;i++){
			cv::drawContours(bgr_copy,contours,number[i],cv::Scalar(0,255,0),3);
		}
		std::vector<cv::Moments> mu(number.size() );
		for( size_t i = 0; i < number.size(); i++ ){
			mu[i] = moments( contours[number[i]], false );
		}
		std::vector<cv::Point2f> mc( number.size() );
		for( size_t i = 0; i < number.size(); i++ ){
			mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
			cv::circle( bgr_copy, mc[i], 3, cv::Scalar(0,0,255), -1, 8, 0 );
		}

		cv::imshow("threshimg",threshimg);
		cv::imshow("canny",canny);
		cv::imshow("bgr",bgr_copy);

		if(key == 's' && number.size() == 4 ){
			for( size_t i = 0; i < number.size(); i++ ){
				if(image_coordinate.size() < 4){
					image_coordinate.push_back(mc[i]);
				}
			}
			cout<<"标定点已保存"<<endl;
		}
		key = cv::waitKey(30);
	}
	np.stop();
	if(image_coordinate.size() != 4){
		cout<<"标定失败"<<endl;
		return;
	}else{
		std::sort(image_coordinate.begin(),image_coordinate.end(),Point2fSort);
		cout<<"标定点为"<<endl;
		cout<<image_coordinate<<endl;
	}
	std::vector<cv::Point3f> world_coordinate;
	world_coordinate.push_back(cv::Point3f(0.362,0.05,0));
	world_coordinate.push_back(cv::Point3f(-0.362,0.05,0));
	world_coordinate.push_back(cv::Point3f(-0.362,0.5,0));
	world_coordinate.push_back(cv::Point3f(0.362,0.5,0));

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << Camera.fx, 0, Camera.cx, 0,Camera.fy, Camera.cy, 0, 0, 1);

	cv::Mat distCoeffs = (cv::Mat_<float>(1,5) << Camera.distCoeffs[0], Camera.distCoeffs[1] ,Camera.distCoeffs[2], Camera.distCoeffs[3],Camera.distCoeffs[4]);
	cv::Mat rvec,tvec,tm;
	cv::solvePnP(world_coordinate,image_coordinate,camera_matrix,distCoeffs,rvec,tvec);
	cv::Rodrigues(rvec,tm);
	Eigen::Matrix4f world2camera(Eigen::Matrix4f::Identity());
	Eigen::Matrix4f camera2world(Eigen::Matrix4f::Identity());
	Eigen::Matrix3f eigen_rota;
	Eigen::Vector3f	eigen_tran;
	world2camera<<tm.at<double>(0,0),tm.at<double>(0,1),tm.at<double>(0,2),tvec.at<double>(0),
			tm.at<double>(1,0),tm.at<double>(1,1),tm.at<double>(1,2),tvec.at<double>(1),
			tm.at<double>(2,0),tm.at<double>(2,1),tm.at<double>(2,2),tvec.at<double>(2),
				0,0,0,1;

	camera2world = world2camera.inverse();
	cout<<"摄像机外参为"<<endl;
	cout<<camera2world<<endl;

}
