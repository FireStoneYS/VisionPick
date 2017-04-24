/*
 * SaveOption.h
 *
 *  Created on: Jan 12, 2017
 *      Author: abds
 */

#ifndef SAVEOPTION_H_
#define SAVEOPTION_H_

#include <opencv/highgui.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define PI 3.1415926

extern Eigen::Matrix4f CAMERA2TABLE;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::Histogram<90> CRH90;
typedef std::pair<std::string, std::vector<float> > vfh_model;

typedef struct TargetPose
{
	int number;
	Eigen::Vector4f centroid,x_axis,y_axis,z_axis;
}TargetPose;

typedef struct BoundingBoxPara
{
	Eigen::Vector3f translation;
	Eigen::Quaternionf rotation;
	double width,height,depth;
}BoundingBoxPara;


typedef struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
    double distCoeffs[5];
}CAMERA_INTRINSIC_PARAMETERS;
extern CAMERA_INTRINSIC_PARAMETERS Camera;

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth);
void PointCloud2Image(PointCloud::Ptr cloudin,cv::Mat& img);

void SaveSinglePitcure(cv::Mat &bgr,cv::Mat &depth,char* name);
void SaveDoublePitcure(cv::Mat &bgr1,cv::Mat &depth1,cv::Mat &bgr2,cv::Mat &depth2,char* name);

void SaveSinglePointCloud(cv::Mat &bgr,cv::Mat &depth,char* name);
void SaveDoublePointCloud(cv::Mat &bgr1,cv::Mat &depth1,cv::Mat &bgr2,cv::Mat &depth2,char* name);
Eigen::Matrix4f Camera2Table();
Eigen::Matrix4f Table2Manipulate();
class ParameterReader
{
	public:
		ParameterReader( std::string filename)
		{
			std::ifstream fin( filename.c_str() );
			if (!fin)
			{
				std::cerr<<"parameter file does not exist."<<std::endl;
				return;
			}
			while(!fin.eof())
			{
				std::string str;
				std::getline( fin, str );
				if (str[0] == '#')
				{
					// 以‘＃’开头的是注释
					continue;
				}

				int pos = str.find("=");
				if (pos == -1)
					continue;
				std::string key = str.substr( 0, pos );
				std::string value = str.substr( pos+1, str.length() );
				data[key] = value;

				if ( !fin.good() )
					break;
			}
		}
		std::string getData( std::string key )
		{
			std::map<std::string, std::string>::iterator iter = data.find(key);
			if (iter == data.end())
			{
				std::cerr<<"Parameter name "<<key<<" not found!"<<std::endl;
				return std::string("NOT_FOUND");
			}
			return iter->second;
		}
	public:
		std::map<std::string, std::string> data;
};
class HypoPara{
	public:
		float InlierThreshold,OcclusionThreshold,Regularizer,RadiusClutter,ClutterRegularizer,RadiusNormals;
		HypoPara(ParameterReader &pd){
			InlierThreshold = atof( pd.getData( "InlierThreshold" ).c_str());
			OcclusionThreshold = atof( pd.getData( "OcclusionThreshold" ).c_str());
			Regularizer = atof( pd.getData( "Regularizer" ).c_str());
			RadiusClutter = atof( pd.getData( "RadiusClutter" ).c_str());
			ClutterRegularizer = atof( pd.getData( "ClutterRegularizer" ).c_str());
			RadiusNormals = atof( pd.getData( "RadiusNormals" ).c_str());
		}

};

#endif /* SAVEOPTION_H_ */
