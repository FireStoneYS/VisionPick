/*
 * SaveOption.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: abds
 */

#include <math.h>
#include "SaveOption.h"


using namespace std;


CAMERA_INTRINSIC_PARAMETERS Camera={
	310.37154441,//cx
	218.82578076,//cy
	517.25336259,//fx
	517.74123176,//fy
	1000.0,//scale
	{-0.0265638,0.1233707,0.00139714,0.00170076,-0.27784827},
};
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / Camera.scale;
            p.x = (n - Camera.cx) * p.z / Camera.fx;
            p.y = (m - Camera.cy) * p.z / Camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的rgb  YS：2017.04.07
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

void PointCloud2Image(PointCloud::Ptr cloudin,cv::Mat& img){
	int m,n;
	for(size_t i = 0; i< cloudin->points.size();i++){

        n = cloudin->points[i].x*Camera.fx/cloudin->points[i].z+Camera.cx;
        n = (n < 639)?n:639;
        n = (n > 0)?n:0;
        m = cloudin->points[i].y*Camera.fy/cloudin->points[i].z+Camera.cy;
        m = (m < 479)?m:479;
        m = (m > 0)?m:0;
        cout<<"n = "<<n<<" m = "<<m<<endl;
        img.ptr<uchar>(m)[n*3] = cloudin->points[i].b;
        img.ptr<uchar>(m)[n*3+1] = cloudin->points[i].g;
        img.ptr<uchar>(m)[n*3+2] = cloudin->points[i].r;

	}

}

void SaveSinglePitcure(cv::Mat &bgr,cv::Mat &depth,char* name){
	char bgr_directory[512];
	char depth_directory[512];
	sprintf(bgr_directory,"/home/abds/study/data/%s_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/%s_depth.png",name);
	cv::imwrite(bgr_directory,bgr);
	cv::imwrite(depth_directory,depth);
	cout<<"Save pitcure success"<<endl;
}

void SaveDoublePitcure(cv::Mat &bgr1,cv::Mat &depth1,cv::Mat &bgr2,cv::Mat &depth2,char* name){
	char bgr_directory[512];
	char depth_directory[512];
	sprintf(bgr_directory,"/home/abds/study/data/%s1_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/%s1_depth.png",name);
	cv::imwrite(bgr_directory,bgr1);
	cv::imwrite(depth_directory,depth1);
	memset(bgr_directory,0,sizeof(bgr_directory));
	memset(depth_directory,0,sizeof(depth_directory));

	sprintf(bgr_directory,"/home/abds/study/data/%s2_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/%s2_depth.png",name);
	cv::imwrite(bgr_directory,bgr2);
	cv::imwrite(depth_directory,depth2);
	cout<<"Save pitcure success"<<endl;
}

void SaveSinglePointCloud(cv::Mat &bgr,cv::Mat &depth,char* name){
	char bgr_directory[512];
	char depth_directory[512];
	sprintf(bgr_directory,"/home/abds/study/data/360/fivebox/%s_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/360/fivebox/%s_depth.png",name);
	cv::imwrite(bgr_directory,bgr);
	cv::imwrite(depth_directory,depth);
	imwrite(bgr_directory,bgr);
	imwrite(depth_directory,depth);
	PointCloud::Ptr cloud1 = image2PointCloud( bgr, depth);
	char directory[512];
	sprintf(directory,"/home/abds/study/data/360/fivebox/%s.pcd",name);
	pcl::io::savePCDFile(directory, *cloud1);
	cout<<"Save single pointcloud success"<<endl;
}


void SaveDoublePointCloud(cv::Mat &bgr1,cv::Mat &depth1,cv::Mat &bgr2,cv::Mat &depth2,char* name){
	char bgr_directory[512];
	char depth_directory[512];
	char directory[512];

	sprintf(bgr_directory,"/home/abds/study/data/%s1_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/%s1_depth.png",name);
	cv::imwrite(bgr_directory,bgr1);
	cv::imwrite(depth_directory,depth1);

	PointCloud::Ptr cloud1 = image2PointCloud( bgr1, depth1);

	sprintf(directory,"/home/abds/study/data/%s1.pcd",name);
	pcl::io::savePCDFile(directory, *cloud1);

	memset(bgr_directory,0,sizeof(bgr_directory));
	memset(depth_directory,0,sizeof(depth_directory));
	memset(directory,0,sizeof(directory));

	sprintf(bgr_directory,"/home/abds/study/data/%s2_bgr.png",name);
	sprintf(depth_directory,"/home/abds/study/data/%s2_depth.png",name);
	cv::imwrite(bgr_directory,bgr2);
	cv::imwrite(depth_directory,depth2);

	PointCloud::Ptr cloud2 = image2PointCloud( bgr2, depth2);
	sprintf(directory,"/home/abds/study/data/%s2.pcd",name);
	pcl::io::savePCDFile(directory, *cloud2);

	cout<<"Save double pointcloud success"<<endl;
}


Eigen::Matrix4f Camera2Table(){
	Eigen::Matrix4f out;
	out<<
			-0.0193834 ,  0.773462 , -0.633547  , 0.559001,
			  0.998929 ,-0.0116506 ,-0.0447859  , 0.297393,
			-0.0420214 , -0.633736 , -0.772407  ,    0.534,
			         0  ,       -0  ,        0  ,        1;
	return out;
}
Eigen::Matrix4f Table2Manipulate(){
	Eigen::Vector3f origin_poistion,right_position,left_position,x_positive,x_negative;
	origin_poistion << 0.315,-0.15,0.24;
	right_position << 0.12,-0.51,0.24;
	left_position << 0.495,0.205,0.24;
	x_positive = right_position - origin_poistion;
	x_negative = left_position - origin_poistion;
	double theta1 = acos(x_positive.dot(Eigen::Vector3f(1,0,0)) /x_positive.norm());
	double theta2 = acos(x_negative.dot(Eigen::Vector3f(-1,0,0)) /x_negative.norm());
	double theta = -(theta1+theta2)/2;
//	cout<<theta<<endl;
	Eigen::AngleAxisf rotation(theta,Eigen::Vector3f(0,0,1));
	Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();
	Eigen::Matrix4f out;
	for(int i = 0 ; i <3 ;i++)
		for(int j = 0 ; j < 3;j++)
			out(i,j)= rotation_matrix(i,j);
	out(0,3) = origin_poistion[0];
	out(1,3) = origin_poistion[1];
	out(2,3) = origin_poistion[2];
	out(3,3) = 1;
	return out;
}

